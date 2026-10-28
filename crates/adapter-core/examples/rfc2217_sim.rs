//! A host-side stand-in for the firmware's bridge: serves sessions on a TCP
//! port with a fake serial port that echoes what it is sent, so that real
//! clients (pyserial's `rfc2217://`, esptool, `telnet`, `nc`) can be run
//! against the Telnet/RFC 2217 state machine without hardware. Every
//! request from the client is printed; the fake port refuses one and a
//! half stop bits so that a refusal can be seen too.
//!
//! ```text
//! cargo run -p adapter-core --example rfc2217_sim -- 2217
//! python -m serial.tools.miniterm rfc2217://localhost:2217 9600
//! ```

use adapter_core::rfc2217::{Command, Session};
use adapter_core::usbserial::{LineSettings, StopBits};
use std::io::{Read, Write};
use std::net::{TcpListener, TcpStream};

fn main() {
    let port: u16 = std::env::args()
        .nth(1)
        .and_then(|p| p.parse().ok())
        .unwrap_or(2217);
    let listener = TcpListener::bind(("127.0.0.1", port)).expect("bind");
    eprintln!("rfc2217_sim: listening on 127.0.0.1:{port}");
    for conn in listener.incoming().flatten() {
        serve(conn);
    }
}

fn serve(mut conn: TcpStream) {
    let peer = conn.peer_addr().map(|a| a.to_string()).unwrap_or_default();
    eprintln!("client {peer} connected");
    let mut session = Session::new(LineSettings::new(115200), "rfc2217_sim");
    let mut buf = [0u8; 512];
    let mut to_serial = Vec::new();
    let mut to_client = Vec::new();
    let mut commands = Vec::new();
    loop {
        let n = match conn.read(&mut buf) {
            Ok(0) | Err(_) => break,
            Ok(n) => n,
        };
        to_serial.clear();
        to_client.clear();
        commands.clear();
        let was_telnet = session.is_telnet();
        session.feed(&buf[..n], &mut to_serial, &mut to_client, &mut commands);
        if !was_telnet && session.is_telnet() {
            eprintln!("  telnet session");
        }
        for &command in &commands {
            let applied = command != Command::StopBits(StopBits::OneAndHalf);
            eprintln!("  {command:?}{}", if applied { "" } else { " (refused)" });
            session.complete(command, applied, &mut to_client);
        }
        if !to_serial.is_empty() {
            eprintln!("  to serial: {:?}", String::from_utf8_lossy(&to_serial));
            // The fake target echoes what it is sent.
            session.encode(&to_serial, &mut to_client);
        }
        if conn.write_all(&to_client).is_err() {
            break;
        }
    }
    eprintln!(
        "client {peer} left; the line was {}{}",
        session.line(),
        if session.is_telnet() {
            " (telnet)"
        } else {
            " (raw)"
        }
    );
}
