//! Telnet (RFC 854) framing with the COM-PORT-OPTION (RFC 2217) on the
//! bridge's TCP port, so that a client can set the baud rate, the framing
//! and the DTR and RTS lines itself. pyserial's `rfc2217://` URLs, esptool's
//! `--port rfc2217://` and a `telnet` that negotiates first speak it. A pure
//! state machine, unit-tested on the host; `examples/rfc2217_sim.rs` runs it
//! against real clients.
//!
//! A session starts undecided. The first byte the client sends settles it:
//! IAC (0xFF), which a client opening with Telnet negotiation starts with,
//! makes the session a Telnet one; anything else keeps the raw bridge, where
//! bytes pass unchanged in both directions for the rest of the session. (A
//! BSD `telnet` waits for the server to negotiate first and so stays raw.)
//!
//! In a Telnet session the bridge offers binary mode both ways, suppress
//! go-ahead and echo (so that `telnet` runs in character mode and leaves
//! echoing to the target), and accepts the client's COM-PORT-OPTION. Each
//! SET-* request becomes a [`Command`] for the bridge to apply to the USB
//! serial port; [`Session::complete`] then answers the client with the value
//! in force, which is how a client learns that a setting was refused.
//! Modem-state and line-state notifications are not sent, and flow control
//! is always reported as none.

use crate::usbserial::{LineSettings, Parity, StopBits};

/// Telnet "interpret as command".
pub const IAC: u8 = 255;
const DONT: u8 = 254;
const DO: u8 = 253;
const WONT: u8 = 252;
const WILL: u8 = 251;
const SB: u8 = 250;
const SE: u8 = 240;

const OPT_BINARY: u8 = 0;
const OPT_ECHO: u8 = 1;
const OPT_SGA: u8 = 3;
const OPT_COM_PORT: u8 = 44;

// COM-PORT-OPTION subcommands from the client. The server answers each
// with the same number plus 100.
const SIGNATURE: u8 = 0;
const SET_BAUDRATE: u8 = 1;
const SET_DATASIZE: u8 = 2;
const SET_PARITY: u8 = 3;
const SET_STOPSIZE: u8 = 4;
const SET_CONTROL: u8 = 5;
const SET_LINESTATE_MASK: u8 = 10;
const SET_MODEMSTATE_MASK: u8 = 11;
const PURGE_DATA: u8 = 12;
const REPLY: u8 = 100;

// SET-CONTROL values.
const FLOW_REQUEST: u8 = 0;
const FLOW_NONE: u8 = 1;
const FLOW_HARDWARE: u8 = 3;
const BREAK_REQUEST: u8 = 4;
const BREAK_ON: u8 = 5;
const BREAK_OFF: u8 = 6;
const DTR_REQUEST: u8 = 7;
const DTR_ON: u8 = 8;
const DTR_OFF: u8 = 9;
const RTS_REQUEST: u8 = 10;
const RTS_ON: u8 = 11;
const RTS_OFF: u8 = 12;
const INBOUND_FLOW_REQUEST: u8 = 13;
const INBOUND_FLOW_NONE: u8 = 14;

/// Longest subnegotiation kept: the client's signature can be anything,
/// every other request is at most a few bytes.
const MAX_SUBNEGOTIATION: usize = 64;

/// Which buffers PURGE-DATA names.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Purge {
    Receive,
    Transmit,
    Both,
}

impl Purge {
    fn code(self) -> u8 {
        match self {
            Purge::Receive => 1,
            Purge::Transmit => 2,
            Purge::Both => 3,
        }
    }
}

/// What the client asked the bridge to do with the serial port. The bridge
/// applies it and reports back with [`Session::complete`], which answers
/// the client.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Command {
    Baud(u32),
    DataBits(u8),
    Parity(Parity),
    StopBits(StopBits),
    Dtr(bool),
    Rts(bool),
    Break(bool),
    Purge(Purge),
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Mode {
    Undecided,
    Raw,
    Telnet,
}

/// Where the parser is in the client's byte stream.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Parse {
    Data,
    /// After IAC.
    Iac,
    /// After IAC WILL/WONT/DO/DONT, waiting for the option.
    Verb(u8),
    /// Inside IAC SB ... IAC SE.
    Sub,
    /// After IAC inside a subnegotiation.
    SubIac,
}

/// One side's state of a Telnet option (RFC 1143 without the queue bit).
#[derive(Clone, Copy, PartialEq, Eq)]
enum Opt {
    No,
    /// We asked for it and wait for the answer.
    WantYes,
    Yes,
}

struct OptionState {
    code: u8,
    /// Whether we are willing to enable it on our side (WILL).
    we_offer: bool,
    /// Whether we accept the client enabling it on its side (DO).
    we_accept: bool,
    /// Our side (WILL/WONT from us, DO/DONT from the client).
    us: Opt,
    /// The client's side (DO/DONT from us, WILL/WONT from the client).
    them: Opt,
}

pub struct Session {
    mode: Mode,
    parse: Parse,
    sub: Vec<u8>,
    sub_overflow: bool,
    options: [OptionState; 4],
    /// The data byte before the current one was CR: an NVT client sends
    /// CR NUL for a bare carriage return.
    after_cr: bool,
    line: LineSettings,
    dtr: bool,
    rts: bool,
    brk: bool,
    signature: String,
}

impl Session {
    /// A session whose serial port starts at `line` with DTR and RTS
    /// asserted, as the bridge opens it. `signature` answers the client's
    /// SIGNATURE request.
    pub fn new(line: LineSettings, signature: &str) -> Session {
        let option = |code, we_offer, we_accept| OptionState {
            code,
            we_offer,
            we_accept,
            us: Opt::No,
            them: Opt::No,
        };
        Session {
            mode: Mode::Undecided,
            parse: Parse::Data,
            sub: Vec::new(),
            sub_overflow: false,
            options: [
                option(OPT_BINARY, true, true),
                option(OPT_SGA, true, true),
                option(OPT_ECHO, true, false),
                option(OPT_COM_PORT, true, true),
            ],
            after_cr: false,
            line,
            dtr: true,
            rts: true,
            brk: false,
            signature: signature.to_string(),
        }
    }

    /// The client opened with Telnet negotiation.
    pub fn is_telnet(&self) -> bool {
        self.mode == Mode::Telnet
    }

    /// The line settings in force, as last acknowledged.
    pub fn line(&self) -> LineSettings {
        self.line
    }

    /// Takes bytes from the client. Serial data goes to `to_serial`,
    /// negotiation answers to `to_client`, and the settings the client asks
    /// for to `commands`, in the order they arrived. The bridge applies each
    /// command and calls [`Session::complete`] with the outcome.
    pub fn feed(
        &mut self,
        input: &[u8],
        to_serial: &mut Vec<u8>,
        to_client: &mut Vec<u8>,
        commands: &mut Vec<Command>,
    ) {
        if self.mode == Mode::Undecided {
            let Some(&first) = input.first() else {
                return;
            };
            if first == IAC {
                self.mode = Mode::Telnet;
                self.offer(to_client);
            } else {
                self.mode = Mode::Raw;
            }
        }
        if self.mode == Mode::Raw {
            to_serial.extend_from_slice(input);
            return;
        }
        for &b in input {
            self.byte(b, to_serial, to_client, commands);
        }
    }

    /// Answers a command the bridge has applied (`applied`) or refused,
    /// with the value now in force.
    pub fn complete(&mut self, command: Command, applied: bool, to_client: &mut Vec<u8>) {
        match command {
            Command::Baud(baud) => {
                if applied {
                    self.line.baud = baud;
                }
                self.reply(SET_BAUDRATE, &self.line.baud.to_be_bytes(), to_client);
            }
            Command::DataBits(bits) => {
                if applied {
                    self.line.data_bits = bits;
                }
                self.reply(SET_DATASIZE, &[self.line.data_bits], to_client);
            }
            Command::Parity(parity) => {
                if applied {
                    self.line.parity = parity;
                }
                self.reply(SET_PARITY, &[self.line.parity.rfc2217()], to_client);
            }
            Command::StopBits(stop_bits) => {
                if applied {
                    self.line.stop_bits = stop_bits;
                }
                self.reply(SET_STOPSIZE, &[self.line.stop_bits.rfc2217()], to_client);
            }
            Command::Dtr(on) => {
                if applied {
                    self.dtr = on;
                }
                self.reply(SET_CONTROL, &[self.dtr_code()], to_client);
            }
            Command::Rts(on) => {
                if applied {
                    self.rts = on;
                }
                self.reply(SET_CONTROL, &[self.rts_code()], to_client);
            }
            Command::Break(on) => {
                if applied {
                    self.brk = on;
                }
                self.reply(SET_CONTROL, &[self.break_code()], to_client);
            }
            Command::Purge(purge) => self.reply(PURGE_DATA, &[purge.code()], to_client),
        }
    }

    /// Frames bytes from the serial port for the client: unchanged in a raw
    /// session; in a Telnet session IAC is doubled and, until the client
    /// has agreed to binary mode from us, a bare CR is sent as CR NUL.
    pub fn encode(&self, from_serial: &[u8], to_client: &mut Vec<u8>) {
        if self.mode != Mode::Telnet {
            to_client.extend_from_slice(from_serial);
            return;
        }
        let binary = self.option(OPT_BINARY).us == Opt::Yes;
        let mut after_cr = false;
        for &b in from_serial {
            if after_cr && b != b'\n' && b != 0 {
                to_client.push(0);
            }
            to_client.push(b);
            if b == IAC {
                to_client.push(IAC);
            }
            after_cr = !binary && b == b'\r';
        }
        if after_cr {
            to_client.push(0);
        }
    }

    /// Our opening offers: binary both ways, suppress go-ahead both ways
    /// and echo from us, so a `telnet` client runs in character mode and
    /// leaves echoing to the target.
    fn offer(&mut self, to_client: &mut Vec<u8>) {
        for (verb, code) in [
            (WILL, OPT_BINARY),
            (DO, OPT_BINARY),
            (WILL, OPT_SGA),
            (DO, OPT_SGA),
            (WILL, OPT_ECHO),
        ] {
            let opt = self.option_mut(code);
            if verb == WILL {
                opt.us = Opt::WantYes;
            } else {
                opt.them = Opt::WantYes;
            }
            to_client.extend_from_slice(&[IAC, verb, code]);
        }
    }

    fn option(&self, code: u8) -> &OptionState {
        self.options
            .iter()
            .find(|o| o.code == code)
            .expect("known option")
    }

    fn option_mut(&mut self, code: u8) -> &mut OptionState {
        self.options
            .iter_mut()
            .find(|o| o.code == code)
            .expect("known option")
    }

    fn byte(
        &mut self,
        b: u8,
        to_serial: &mut Vec<u8>,
        to_client: &mut Vec<u8>,
        commands: &mut Vec<Command>,
    ) {
        match self.parse {
            Parse::Data => {
                let binary = self.option(OPT_BINARY).them == Opt::Yes;
                match b {
                    IAC => self.parse = Parse::Iac,
                    // CR NUL is how an NVT client sends a carriage return.
                    0 if self.after_cr && !binary => {}
                    _ => to_serial.push(b),
                }
                self.after_cr = b == b'\r';
            }
            Parse::Iac => match b {
                IAC => {
                    to_serial.push(IAC);
                    self.parse = Parse::Data;
                }
                WILL | WONT | DO | DONT => self.parse = Parse::Verb(b),
                SB => {
                    self.sub.clear();
                    self.sub_overflow = false;
                    self.parse = Parse::Sub;
                }
                // NOP, AYT, BRK and the other two-byte commands: nothing to do.
                _ => self.parse = Parse::Data,
            },
            Parse::Verb(verb) => {
                self.negotiate(verb, b, to_client);
                self.parse = Parse::Data;
            }
            Parse::Sub => {
                if b == IAC {
                    self.parse = Parse::SubIac;
                } else {
                    self.sub_push(b);
                }
            }
            Parse::SubIac => match b {
                IAC => {
                    self.sub_push(IAC);
                    self.parse = Parse::Sub;
                }
                SE => {
                    if !self.sub_overflow {
                        let sub = std::mem::take(&mut self.sub);
                        self.subnegotiation(&sub, to_client, commands);
                        self.sub = sub;
                    }
                    self.parse = Parse::Data;
                }
                // Not a subnegotiation after all; drop what was collected.
                _ => self.parse = Parse::Data,
            },
        }
    }

    fn sub_push(&mut self, b: u8) {
        if self.sub.len() < MAX_SUBNEGOTIATION {
            self.sub.push(b);
        } else {
            self.sub_overflow = true;
        }
    }

    /// Answers a WILL/WONT/DO/DONT from the client. A request for an option
    /// we offered or accept is acknowledged once; anything else is refused.
    fn negotiate(&mut self, verb: u8, code: u8, to_client: &mut Vec<u8>) {
        let Some(opt) = self.options.iter_mut().find(|o| o.code == code) else {
            match verb {
                DO => to_client.extend_from_slice(&[IAC, WONT, code]),
                WILL => to_client.extend_from_slice(&[IAC, DONT, code]),
                _ => {}
            }
            return;
        };
        match verb {
            DO => match opt.us {
                Opt::WantYes => opt.us = Opt::Yes,
                Opt::No if opt.we_offer => {
                    opt.us = Opt::Yes;
                    to_client.extend_from_slice(&[IAC, WILL, code]);
                }
                Opt::No => to_client.extend_from_slice(&[IAC, WONT, code]),
                Opt::Yes => {}
            },
            DONT => match opt.us {
                Opt::WantYes => opt.us = Opt::No,
                Opt::Yes => {
                    opt.us = Opt::No;
                    to_client.extend_from_slice(&[IAC, WONT, code]);
                }
                Opt::No => {}
            },
            WILL => match opt.them {
                Opt::WantYes => opt.them = Opt::Yes,
                Opt::No if opt.we_accept => {
                    opt.them = Opt::Yes;
                    to_client.extend_from_slice(&[IAC, DO, code]);
                }
                Opt::No => to_client.extend_from_slice(&[IAC, DONT, code]),
                Opt::Yes => {}
            },
            _ => match opt.them {
                Opt::WantYes => opt.them = Opt::No,
                Opt::Yes => {
                    opt.them = Opt::No;
                    to_client.extend_from_slice(&[IAC, DONT, code]);
                }
                Opt::No => {}
            },
        }
    }

    /// Handles one COM-PORT-OPTION subnegotiation. Queries are answered
    /// here; settings become commands and are answered by `complete`.
    fn subnegotiation(&mut self, sub: &[u8], to_client: &mut Vec<u8>, commands: &mut Vec<Command>) {
        let [OPT_COM_PORT, command, value @ ..] = sub else {
            return;
        };
        match *command {
            SIGNATURE => {
                // An empty signature asks for ours; a non-empty one is the
                // client introducing itself.
                if value.is_empty() {
                    let signature = self.signature.clone();
                    self.reply(SIGNATURE, signature.as_bytes(), to_client);
                }
            }
            SET_BAUDRATE => {
                let Ok(bytes) = <[u8; 4]>::try_from(value) else {
                    return;
                };
                match u32::from_be_bytes(bytes) {
                    0 => self.reply(SET_BAUDRATE, &self.line.baud.to_be_bytes(), to_client),
                    baud => commands.push(Command::Baud(baud)),
                }
            }
            SET_DATASIZE => match value {
                [0] => self.reply(SET_DATASIZE, &[self.line.data_bits], to_client),
                [bits @ 5..=8] => commands.push(Command::DataBits(*bits)),
                [_] => self.reply(SET_DATASIZE, &[self.line.data_bits], to_client),
                _ => {}
            },
            SET_PARITY => match value {
                [0] => self.reply(SET_PARITY, &[self.line.parity.rfc2217()], to_client),
                [code] => match Parity::from_rfc2217(*code) {
                    Some(parity) => commands.push(Command::Parity(parity)),
                    None => self.reply(SET_PARITY, &[self.line.parity.rfc2217()], to_client),
                },
                _ => {}
            },
            SET_STOPSIZE => match value {
                [0] => self.reply(SET_STOPSIZE, &[self.line.stop_bits.rfc2217()], to_client),
                [code] => match StopBits::from_rfc2217(*code) {
                    Some(stop_bits) => commands.push(Command::StopBits(stop_bits)),
                    None => self.reply(SET_STOPSIZE, &[self.line.stop_bits.rfc2217()], to_client),
                },
                _ => {}
            },
            SET_CONTROL => {
                let [code] = value else {
                    return;
                };
                match *code {
                    // Flow control is never done; a request for one is
                    // answered with "none", which tells the client it was
                    // refused.
                    FLOW_REQUEST..=FLOW_HARDWARE => {
                        self.reply(SET_CONTROL, &[FLOW_NONE], to_client)
                    }
                    BREAK_REQUEST => self.reply(SET_CONTROL, &[self.break_code()], to_client),
                    BREAK_ON => commands.push(Command::Break(true)),
                    BREAK_OFF => commands.push(Command::Break(false)),
                    DTR_REQUEST => self.reply(SET_CONTROL, &[self.dtr_code()], to_client),
                    DTR_ON => commands.push(Command::Dtr(true)),
                    DTR_OFF => commands.push(Command::Dtr(false)),
                    RTS_REQUEST => self.reply(SET_CONTROL, &[self.rts_code()], to_client),
                    RTS_ON => commands.push(Command::Rts(true)),
                    RTS_OFF => commands.push(Command::Rts(false)),
                    INBOUND_FLOW_REQUEST.. => {
                        self.reply(SET_CONTROL, &[INBOUND_FLOW_NONE], to_client)
                    }
                }
            }
            // No notifications are sent, so no mask is honoured.
            SET_LINESTATE_MASK => self.reply(SET_LINESTATE_MASK, &[0], to_client),
            SET_MODEMSTATE_MASK => self.reply(SET_MODEMSTATE_MASK, &[0], to_client),
            PURGE_DATA => {
                let purge = match value {
                    [1] => Purge::Receive,
                    [2] => Purge::Transmit,
                    [3] => Purge::Both,
                    _ => return,
                };
                commands.push(Command::Purge(purge));
            }
            // NOTIFY-LINESTATE and NOTIFY-MODEMSTATE requests (no state is
            // tracked) and FLOWCONTROL-SUSPEND/RESUME (data is not held back).
            _ => {}
        }
    }

    fn dtr_code(&self) -> u8 {
        if self.dtr {
            DTR_ON
        } else {
            DTR_OFF
        }
    }

    fn rts_code(&self) -> u8 {
        if self.rts {
            RTS_ON
        } else {
            RTS_OFF
        }
    }

    fn break_code(&self) -> u8 {
        if self.brk {
            BREAK_ON
        } else {
            BREAK_OFF
        }
    }

    /// `IAC SB COM-PORT-OPTION (command + 100) value IAC SE`, with IAC in
    /// the value doubled.
    fn reply(&self, command: u8, value: &[u8], to_client: &mut Vec<u8>) {
        to_client.extend_from_slice(&[IAC, SB, OPT_COM_PORT, command + REPLY]);
        for &b in value {
            to_client.push(b);
            if b == IAC {
                to_client.push(IAC);
            }
        }
        to_client.extend_from_slice(&[IAC, SE]);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    struct Harness {
        session: Session,
        to_serial: Vec<u8>,
        to_client: Vec<u8>,
        commands: Vec<Command>,
    }

    impl Harness {
        fn new() -> Harness {
            Harness {
                session: Session::new(LineSettings::new(115200), "test"),
                to_serial: Vec::new(),
                to_client: Vec::new(),
                commands: Vec::new(),
            }
        }

        /// Feeds `input` and returns what went to the client.
        fn feed(&mut self, input: &[u8]) -> Vec<u8> {
            self.to_client.clear();
            self.session.feed(
                input,
                &mut self.to_serial,
                &mut self.to_client,
                &mut self.commands,
            );
            self.to_client.clone()
        }

        fn complete(&mut self, command: Command, applied: bool) -> Vec<u8> {
            self.to_client.clear();
            self.session.complete(command, applied, &mut self.to_client);
            self.to_client.clone()
        }

        fn encode(&self, from_serial: &[u8]) -> Vec<u8> {
            let mut out = Vec::new();
            self.session.encode(from_serial, &mut out);
            out
        }
    }

    /// pyserial's opening negotiation: DO ECHO, WILL SGA, DO SGA, DO
    /// COM-PORT-OPTION, WILL COM-PORT-OPTION.
    const PYSERIAL_OPEN: [u8; 15] = [
        IAC,
        DO,
        OPT_ECHO,
        IAC,
        WILL,
        OPT_SGA,
        IAC,
        DO,
        OPT_SGA,
        IAC,
        DO,
        OPT_COM_PORT,
        IAC,
        WILL,
        OPT_COM_PORT,
    ];
    const OUR_OFFER: [u8; 15] = [
        IAC, WILL, OPT_BINARY, IAC, DO, OPT_BINARY, IAC, WILL, OPT_SGA, IAC, DO, OPT_SGA, IAC,
        WILL, OPT_ECHO,
    ];

    fn sub(command: u8, value: &[u8]) -> Vec<u8> {
        let mut v = vec![IAC, SB, OPT_COM_PORT, command];
        v.extend_from_slice(value);
        v.extend_from_slice(&[IAC, SE]);
        v
    }

    fn reply(command: u8, value: &[u8]) -> Vec<u8> {
        sub(command + REPLY, value)
    }

    #[test]
    fn a_client_that_does_not_start_with_iac_stays_raw() {
        let mut h = Harness::new();
        assert!(h.feed(b"ls\r").is_empty());
        assert!(!h.session.is_telnet());
        // Even IAC is data now, and nothing is doubled on the way back.
        assert!(h.feed(&[IAC, DO, OPT_ECHO]).is_empty());
        assert_eq!(
            h.to_serial,
            [b"ls\r".as_slice(), &[IAC, DO, OPT_ECHO]].concat()
        );
        assert!(h.commands.is_empty());
        assert_eq!(h.encode(&[b'a', IAC, b'\r']), [b'a', IAC, b'\r']);
    }

    #[test]
    fn pyserial_open_is_answered_once_per_option() {
        let mut h = Harness::new();
        let out = h.feed(&PYSERIAL_OPEN);
        assert!(h.session.is_telnet());
        // Our offers first, then answers only to what we did not already
        // offer: the client's COM-PORT-OPTION both ways.
        let mut expected = OUR_OFFER.to_vec();
        expected.extend_from_slice(&[IAC, WILL, OPT_COM_PORT, IAC, DO, OPT_COM_PORT]);
        assert_eq!(out, expected);
        // The client's acknowledgements of our offers get no answer.
        assert!(h
            .feed(&[IAC, WILL, OPT_BINARY, IAC, DO, OPT_BINARY, IAC, DO, OPT_SGA])
            .is_empty());
        // A repeated DO is not answered again.
        assert!(h.feed(&[IAC, DO, OPT_ECHO]).is_empty());
        assert!(h.to_serial.is_empty());
    }

    #[test]
    fn unknown_options_are_refused_and_echo_from_the_client_too() {
        let mut h = Harness::new();
        h.feed(&[IAC, DO, OPT_SGA]);
        assert_eq!(h.feed(&[IAC, DO, 24]), [IAC, WONT, 24]);
        assert_eq!(h.feed(&[IAC, WILL, 31]), [IAC, DONT, 31]);
        assert_eq!(h.feed(&[IAC, WILL, OPT_ECHO]), [IAC, DONT, OPT_ECHO]);
        assert!(h.feed(&[IAC, WONT, 31, IAC, DONT, 24]).is_empty());
    }

    #[test]
    fn baud_rate_is_a_command_and_the_answer_carries_the_result() {
        let mut h = Harness::new();
        h.feed(&PYSERIAL_OPEN);
        assert!(h
            .feed(&sub(SET_BAUDRATE, &9600u32.to_be_bytes()))
            .is_empty());
        assert_eq!(h.commands, [Command::Baud(9600)]);
        assert_eq!(
            h.complete(Command::Baud(9600), true),
            reply(SET_BAUDRATE, &9600u32.to_be_bytes())
        );
        assert_eq!(h.session.line().baud, 9600);
        // A query answers the rate in force without a command.
        h.commands.clear();
        assert_eq!(
            h.feed(&sub(SET_BAUDRATE, &[0, 0, 0, 0])),
            reply(SET_BAUDRATE, &9600u32.to_be_bytes())
        );
        assert!(h.commands.is_empty());
        // A refused change answers the unchanged rate.
        assert_eq!(
            h.complete(Command::Baud(50), false),
            reply(SET_BAUDRATE, &9600u32.to_be_bytes())
        );
    }

    #[test]
    fn framing_commands_validate_their_values() {
        let mut h = Harness::new();
        h.feed(&PYSERIAL_OPEN);
        let mut input = sub(SET_DATASIZE, &[7]);
        input.extend(sub(SET_PARITY, &[3]));
        input.extend(sub(SET_STOPSIZE, &[2]));
        assert!(h.feed(&input).is_empty());
        assert_eq!(
            h.commands,
            [
                Command::DataBits(7),
                Command::Parity(Parity::Even),
                Command::StopBits(StopBits::Two)
            ]
        );
        h.commands.clear();
        // 9 data bits, parity 6 and stop size 4 do not exist: answered with
        // the current values, no command.
        let mut input = sub(SET_DATASIZE, &[9]);
        input.extend(sub(SET_PARITY, &[6]));
        input.extend(sub(SET_STOPSIZE, &[4]));
        let expected = [
            reply(SET_DATASIZE, &[8]),
            reply(SET_PARITY, &[1]),
            reply(SET_STOPSIZE, &[1]),
        ]
        .concat();
        assert_eq!(h.feed(&input), expected);
        assert!(h.commands.is_empty());
        assert_eq!(
            h.complete(Command::DataBits(7), true),
            reply(SET_DATASIZE, &[7])
        );
        assert_eq!(
            h.complete(Command::StopBits(StopBits::OneAndHalf), false),
            reply(SET_STOPSIZE, &[1])
        );
    }

    #[test]
    fn control_lines_and_break() {
        let mut h = Harness::new();
        h.feed(&PYSERIAL_OPEN);
        // Both lines start asserted, as the bridge opens the port.
        assert_eq!(
            h.feed(&sub(SET_CONTROL, &[DTR_REQUEST])),
            reply(SET_CONTROL, &[DTR_ON])
        );
        assert_eq!(
            h.feed(&sub(SET_CONTROL, &[RTS_REQUEST])),
            reply(SET_CONTROL, &[RTS_ON])
        );
        // esptool's reset sequence.
        let mut input = sub(SET_CONTROL, &[DTR_OFF]);
        input.extend(sub(SET_CONTROL, &[RTS_ON]));
        assert!(h.feed(&input).is_empty());
        assert_eq!(h.commands, [Command::Dtr(false), Command::Rts(true)]);
        assert_eq!(
            h.complete(Command::Dtr(false), true),
            reply(SET_CONTROL, &[DTR_OFF])
        );
        assert_eq!(
            h.complete(Command::Rts(true), true),
            reply(SET_CONTROL, &[RTS_ON])
        );
        assert_eq!(
            h.feed(&sub(SET_CONTROL, &[DTR_REQUEST])),
            reply(SET_CONTROL, &[DTR_OFF])
        );
        h.commands.clear();
        assert!(h.feed(&sub(SET_CONTROL, &[BREAK_ON])).is_empty());
        assert_eq!(h.commands, [Command::Break(true)]);
        // A chip without break keeps it off.
        assert_eq!(
            h.complete(Command::Break(true), false),
            reply(SET_CONTROL, &[BREAK_OFF])
        );
        assert_eq!(
            h.complete(Command::Break(true), true),
            reply(SET_CONTROL, &[BREAK_ON])
        );
        assert_eq!(
            h.feed(&sub(SET_CONTROL, &[BREAK_REQUEST])),
            reply(SET_CONTROL, &[BREAK_ON])
        );
    }

    #[test]
    fn flow_control_is_always_none() {
        let mut h = Harness::new();
        h.feed(&PYSERIAL_OPEN);
        assert_eq!(
            h.feed(&sub(SET_CONTROL, &[FLOW_NONE])),
            reply(SET_CONTROL, &[FLOW_NONE])
        );
        // Hardware flow control is answered with "none": the client sees
        // its request refused.
        assert_eq!(
            h.feed(&sub(SET_CONTROL, &[FLOW_HARDWARE])),
            reply(SET_CONTROL, &[FLOW_NONE])
        );
        assert_eq!(
            h.feed(&sub(SET_CONTROL, &[FLOW_REQUEST])),
            reply(SET_CONTROL, &[FLOW_NONE])
        );
        assert_eq!(
            h.feed(&sub(SET_CONTROL, &[16])),
            reply(SET_CONTROL, &[INBOUND_FLOW_NONE])
        );
        assert!(h.commands.is_empty());
    }

    #[test]
    fn purge_masks_and_signature() {
        let mut h = Harness::new();
        h.feed(&PYSERIAL_OPEN);
        assert!(h.feed(&sub(PURGE_DATA, &[3])).is_empty());
        assert_eq!(h.commands, [Command::Purge(Purge::Both)]);
        assert_eq!(
            h.complete(Command::Purge(Purge::Both), true),
            reply(PURGE_DATA, &[3])
        );
        assert_eq!(
            h.feed(&sub(SET_MODEMSTATE_MASK, &[0x7f])),
            reply(SET_MODEMSTATE_MASK, &[0])
        );
        assert_eq!(
            h.feed(&sub(SET_LINESTATE_MASK, &[0x7f])),
            reply(SET_LINESTATE_MASK, &[0])
        );
        assert_eq!(h.feed(&sub(SIGNATURE, &[])), reply(SIGNATURE, b"test"));
        // The client's own signature is not answered.
        assert!(h.feed(&sub(SIGNATURE, b"pyserial")).is_empty());
        // Notify requests and flow-control suspend/resume are ignored.
        assert!(h.feed(&sub(7, &[])).is_empty());
        assert!(h.feed(&sub(8, &[])).is_empty());
        assert!(h.to_serial.is_empty());
    }

    #[test]
    fn iac_is_escaped_in_both_directions_and_across_chunks() {
        let mut h = Harness::new();
        h.feed(&PYSERIAL_OPEN);
        // A baud rate of 255 carries an IAC, doubled on the wire both ways.
        let mut input = sub(SET_BAUDRATE, &[0, 0, 0, IAC, IAC]);
        input.extend([b'a', IAC, IAC, b'b', IAC, 241 /* NOP */, b'c']);
        // Split at awkward points: inside the subnegotiation, after an IAC.
        let (first, rest) = input.split_at(7);
        let (second, third) = rest.split_at(8);
        assert!(h.feed(first).is_empty());
        assert!(h.feed(second).is_empty());
        assert!(h.feed(third).is_empty());
        assert_eq!(h.commands, [Command::Baud(255)]);
        assert_eq!(h.to_serial, [b'a', IAC, b'b', b'c']);
        assert_eq!(
            h.complete(Command::Baud(255), true),
            reply(SET_BAUDRATE, &[0, 0, 0, IAC, IAC])
        );
        assert_eq!(h.encode(&[b'x', IAC]), [b'x', IAC, IAC]);
    }

    #[test]
    fn carriage_returns_follow_binary_mode() {
        let mut h = Harness::new();
        // A telnet client that negotiates nothing but SGA.
        h.feed(&[IAC, DO, OPT_SGA]);
        // NVT: CR NUL from the client is a bare CR; a bare CR to the client
        // becomes CR NUL, while CR LF stays as it is.
        h.feed(&[b'l', b's', b'\r', 0]);
        assert_eq!(h.to_serial, b"ls\r");
        assert_eq!(h.encode(b"50%\rdone\r\n"), b"50%\r\0done\r\n");
        // Once binary is on both ways, bytes pass as they are.
        assert!(h
            .feed(&[IAC, WILL, OPT_BINARY, IAC, DO, OPT_BINARY])
            .is_empty());
        h.to_serial.clear();
        h.feed(&[b'\r', 0]);
        assert_eq!(h.to_serial, [b'\r', 0]);
        assert_eq!(h.encode(b"50%\r"), b"50%\r");
    }

    #[test]
    fn oversized_and_malformed_subnegotiations_are_dropped() {
        let mut h = Harness::new();
        h.feed(&PYSERIAL_OPEN);
        let mut input = vec![IAC, SB, OPT_COM_PORT, SIGNATURE];
        input.extend(std::iter::repeat_n(b'x', MAX_SUBNEGOTIATION + 10));
        input.extend([IAC, SE]);
        assert!(h.feed(&input).is_empty());
        // IAC followed by something other than IAC or SE ends the
        // subnegotiation without acting on it; the next one works.
        assert!(h
            .feed(&[IAC, SB, OPT_COM_PORT, SET_BAUDRATE, 0, 0, 0, 0, IAC, 241])
            .is_empty());
        assert_eq!(
            h.feed(&sub(SET_BAUDRATE, &[0, 0, 0, 0])),
            reply(SET_BAUDRATE, &115200u32.to_be_bytes())
        );
        // A subnegotiation for another option is ignored.
        assert!(h.feed(&[IAC, SB, 24, 1, IAC, SE]).is_empty());
        assert!(h.commands.is_empty());
        assert!(h.to_serial.is_empty());
    }
}
