//! Platform-independent logic for the adapter firmware, kept separate so it can
//! be unit-tested on the host: the interactive setup shell, the line-setting
//! encodings of USB-serial chips, the reading of USB descriptors and the
//! Telnet/RFC 2217 state machine of the bridge.
pub mod rfc2217;
pub mod shell;
pub mod usbdesc;
pub mod usbserial;
