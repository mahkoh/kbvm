use crate::Keycode;

/// `ESC = 1`
pub const ESC: Keycode = Keycode::from_evdev(1);
/// `1 = 2`
pub const _1: Keycode = Keycode::from_evdev(2);
/// `2 = 3`
pub const _2: Keycode = Keycode::from_evdev(3);
/// `3 = 4`
pub const _3: Keycode = Keycode::from_evdev(4);
/// `4 = 5`
pub const _4: Keycode = Keycode::from_evdev(5);
/// `5 = 6`
pub const _5: Keycode = Keycode::from_evdev(6);
/// `6 = 7`
pub const _6: Keycode = Keycode::from_evdev(7);
/// `7 = 8`
pub const _7: Keycode = Keycode::from_evdev(8);
/// `8 = 9`
pub const _8: Keycode = Keycode::from_evdev(9);
/// `9 = 10`
pub const _9: Keycode = Keycode::from_evdev(10);
/// `0 = 11`
pub const _0: Keycode = Keycode::from_evdev(11);
/// `MINUS = 12`
pub const MINUS: Keycode = Keycode::from_evdev(12);
/// `EQUAL = 13`
pub const EQUAL: Keycode = Keycode::from_evdev(13);
/// `BACKSPACE = 14`
pub const BACKSPACE: Keycode = Keycode::from_evdev(14);
/// `TAB = 15`
pub const TAB: Keycode = Keycode::from_evdev(15);
/// `Q = 16`
pub const Q: Keycode = Keycode::from_evdev(16);
/// `W = 17`
pub const W: Keycode = Keycode::from_evdev(17);
/// `E = 18`
pub const E: Keycode = Keycode::from_evdev(18);
/// `R = 19`
pub const R: Keycode = Keycode::from_evdev(19);
/// `T = 20`
pub const T: Keycode = Keycode::from_evdev(20);
/// `Y = 21`
pub const Y: Keycode = Keycode::from_evdev(21);
/// `U = 22`
pub const U: Keycode = Keycode::from_evdev(22);
/// `I = 23`
pub const I: Keycode = Keycode::from_evdev(23);
/// `O = 24`
pub const O: Keycode = Keycode::from_evdev(24);
/// `P = 25`
pub const P: Keycode = Keycode::from_evdev(25);
/// `LEFTBRACE = 26`
pub const LEFTBRACE: Keycode = Keycode::from_evdev(26);
/// `RIGHTBRACE = 27`
pub const RIGHTBRACE: Keycode = Keycode::from_evdev(27);
/// `ENTER = 28`
pub const ENTER: Keycode = Keycode::from_evdev(28);
/// `LEFTCTRL = 29`
pub const LEFTCTRL: Keycode = Keycode::from_evdev(29);
/// `A = 30`
pub const A: Keycode = Keycode::from_evdev(30);
/// `S = 31`
pub const S: Keycode = Keycode::from_evdev(31);
/// `D = 32`
pub const D: Keycode = Keycode::from_evdev(32);
/// `F = 33`
pub const F: Keycode = Keycode::from_evdev(33);
/// `G = 34`
pub const G: Keycode = Keycode::from_evdev(34);
/// `H = 35`
pub const H: Keycode = Keycode::from_evdev(35);
/// `J = 36`
pub const J: Keycode = Keycode::from_evdev(36);
/// `K = 37`
pub const K: Keycode = Keycode::from_evdev(37);
/// `L = 38`
pub const L: Keycode = Keycode::from_evdev(38);
/// `SEMICOLON = 39`
pub const SEMICOLON: Keycode = Keycode::from_evdev(39);
/// `APOSTROPHE = 40`
pub const APOSTROPHE: Keycode = Keycode::from_evdev(40);
/// `GRAVE = 41`
pub const GRAVE: Keycode = Keycode::from_evdev(41);
/// `LEFTSHIFT = 42`
pub const LEFTSHIFT: Keycode = Keycode::from_evdev(42);
/// `BACKSLASH = 43`
pub const BACKSLASH: Keycode = Keycode::from_evdev(43);
/// `Z = 44`
pub const Z: Keycode = Keycode::from_evdev(44);
/// `X = 45`
pub const X: Keycode = Keycode::from_evdev(45);
/// `C = 46`
pub const C: Keycode = Keycode::from_evdev(46);
/// `V = 47`
pub const V: Keycode = Keycode::from_evdev(47);
/// `B = 48`
pub const B: Keycode = Keycode::from_evdev(48);
/// `N = 49`
pub const N: Keycode = Keycode::from_evdev(49);
/// `M = 50`
pub const M: Keycode = Keycode::from_evdev(50);
/// `COMMA = 51`
pub const COMMA: Keycode = Keycode::from_evdev(51);
/// `DOT = 52`
pub const DOT: Keycode = Keycode::from_evdev(52);
/// `SLASH = 53`
pub const SLASH: Keycode = Keycode::from_evdev(53);
/// `RIGHTSHIFT = 54`
pub const RIGHTSHIFT: Keycode = Keycode::from_evdev(54);
/// `KPASTERISK = 55`
pub const KPASTERISK: Keycode = Keycode::from_evdev(55);
/// `LEFTALT = 56`
pub const LEFTALT: Keycode = Keycode::from_evdev(56);
/// `SPACE = 57`
pub const SPACE: Keycode = Keycode::from_evdev(57);
/// `CAPSLOCK = 58`
pub const CAPSLOCK: Keycode = Keycode::from_evdev(58);
/// `F1 = 59`
pub const F1: Keycode = Keycode::from_evdev(59);
/// `F2 = 60`
pub const F2: Keycode = Keycode::from_evdev(60);
/// `F3 = 61`
pub const F3: Keycode = Keycode::from_evdev(61);
/// `F4 = 62`
pub const F4: Keycode = Keycode::from_evdev(62);
/// `F5 = 63`
pub const F5: Keycode = Keycode::from_evdev(63);
/// `F6 = 64`
pub const F6: Keycode = Keycode::from_evdev(64);
/// `F7 = 65`
pub const F7: Keycode = Keycode::from_evdev(65);
/// `F8 = 66`
pub const F8: Keycode = Keycode::from_evdev(66);
/// `F9 = 67`
pub const F9: Keycode = Keycode::from_evdev(67);
/// `F10 = 68`
pub const F10: Keycode = Keycode::from_evdev(68);
/// `NUMLOCK = 69`
pub const NUMLOCK: Keycode = Keycode::from_evdev(69);
/// `SCROLLLOCK = 70`
pub const SCROLLLOCK: Keycode = Keycode::from_evdev(70);
/// `KP7 = 71`
pub const KP7: Keycode = Keycode::from_evdev(71);
/// `KP8 = 72`
pub const KP8: Keycode = Keycode::from_evdev(72);
/// `KP9 = 73`
pub const KP9: Keycode = Keycode::from_evdev(73);
/// `KPMINUS = 74`
pub const KPMINUS: Keycode = Keycode::from_evdev(74);
/// `KP4 = 75`
pub const KP4: Keycode = Keycode::from_evdev(75);
/// `KP5 = 76`
pub const KP5: Keycode = Keycode::from_evdev(76);
/// `KP6 = 77`
pub const KP6: Keycode = Keycode::from_evdev(77);
/// `KPPLUS = 78`
pub const KPPLUS: Keycode = Keycode::from_evdev(78);
/// `KP1 = 79`
pub const KP1: Keycode = Keycode::from_evdev(79);
/// `KP2 = 80`
pub const KP2: Keycode = Keycode::from_evdev(80);
/// `KP3 = 81`
pub const KP3: Keycode = Keycode::from_evdev(81);
/// `KP0 = 82`
pub const KP0: Keycode = Keycode::from_evdev(82);
/// `KPDOT = 83`
pub const KPDOT: Keycode = Keycode::from_evdev(83);
/// `ZENKAKUHANKAKU = 85`
pub const ZENKAKUHANKAKU: Keycode = Keycode::from_evdev(85);
/// `102ND = 86`
pub const _102ND: Keycode = Keycode::from_evdev(86);
/// `F11 = 87`
pub const F11: Keycode = Keycode::from_evdev(87);
/// `F12 = 88`
pub const F12: Keycode = Keycode::from_evdev(88);
/// `RO = 89`
pub const RO: Keycode = Keycode::from_evdev(89);
/// `KATAKANA = 90`
pub const KATAKANA: Keycode = Keycode::from_evdev(90);
/// `HIRAGANA = 91`
pub const HIRAGANA: Keycode = Keycode::from_evdev(91);
/// `HENKAN = 92`
pub const HENKAN: Keycode = Keycode::from_evdev(92);
/// `KATAKANAHIRAGANA = 93`
pub const KATAKANAHIRAGANA: Keycode = Keycode::from_evdev(93);
/// `MUHENKAN = 94`
pub const MUHENKAN: Keycode = Keycode::from_evdev(94);
/// `KPJPCOMMA = 95`
pub const KPJPCOMMA: Keycode = Keycode::from_evdev(95);
/// `KPENTER = 96`
pub const KPENTER: Keycode = Keycode::from_evdev(96);
/// `RIGHTCTRL = 97`
pub const RIGHTCTRL: Keycode = Keycode::from_evdev(97);
/// `KPSLASH = 98`
pub const KPSLASH: Keycode = Keycode::from_evdev(98);
/// `SYSRQ = 99`
pub const SYSRQ: Keycode = Keycode::from_evdev(99);
/// `RIGHTALT = 100`
pub const RIGHTALT: Keycode = Keycode::from_evdev(100);
/// `LINEFEED = 101`
pub const LINEFEED: Keycode = Keycode::from_evdev(101);
/// `HOME = 102`
pub const HOME: Keycode = Keycode::from_evdev(102);
/// `UP = 103`
pub const UP: Keycode = Keycode::from_evdev(103);
/// `PAGEUP = 104`
pub const PAGEUP: Keycode = Keycode::from_evdev(104);
/// `LEFT = 105`
pub const LEFT: Keycode = Keycode::from_evdev(105);
/// `RIGHT = 106`
pub const RIGHT: Keycode = Keycode::from_evdev(106);
/// `END = 107`
pub const END: Keycode = Keycode::from_evdev(107);
/// `DOWN = 108`
pub const DOWN: Keycode = Keycode::from_evdev(108);
/// `PAGEDOWN = 109`
pub const PAGEDOWN: Keycode = Keycode::from_evdev(109);
/// `INSERT = 110`
pub const INSERT: Keycode = Keycode::from_evdev(110);
/// `DELETE = 111`
pub const DELETE: Keycode = Keycode::from_evdev(111);
/// `MACRO = 112`
pub const MACRO: Keycode = Keycode::from_evdev(112);
/// `MUTE = 113`
pub const MUTE: Keycode = Keycode::from_evdev(113);
/// `VOLUMEDOWN = 114`
pub const VOLUMEDOWN: Keycode = Keycode::from_evdev(114);
/// `VOLUMEUP = 115`
pub const VOLUMEUP: Keycode = Keycode::from_evdev(115);
/// `POWER = 116`
pub const POWER: Keycode = Keycode::from_evdev(116);
/// `KPEQUAL = 117`
pub const KPEQUAL: Keycode = Keycode::from_evdev(117);
/// `KPPLUSMINUS = 118`
pub const KPPLUSMINUS: Keycode = Keycode::from_evdev(118);
/// `PAUSE = 119`
pub const PAUSE: Keycode = Keycode::from_evdev(119);
/// `SCALE = 120`
pub const SCALE: Keycode = Keycode::from_evdev(120);
/// `KPCOMMA = 121`
pub const KPCOMMA: Keycode = Keycode::from_evdev(121);
/// `HANGEUL = 122`
pub const HANGEUL: Keycode = Keycode::from_evdev(122);
/// `HANGUEL = 122`
pub const HANGUEL: Keycode = Keycode::from_evdev(122);
/// `HANJA = 123`
pub const HANJA: Keycode = Keycode::from_evdev(123);
/// `YEN = 124`
pub const YEN: Keycode = Keycode::from_evdev(124);
/// `LEFTMETA = 125`
pub const LEFTMETA: Keycode = Keycode::from_evdev(125);
/// `RIGHTMETA = 126`
pub const RIGHTMETA: Keycode = Keycode::from_evdev(126);
/// `COMPOSE = 127`
pub const COMPOSE: Keycode = Keycode::from_evdev(127);
/// `STOP = 128`
pub const STOP: Keycode = Keycode::from_evdev(128);
/// `AGAIN = 129`
pub const AGAIN: Keycode = Keycode::from_evdev(129);
/// `PROPS = 130`
pub const PROPS: Keycode = Keycode::from_evdev(130);
/// `UNDO = 131`
pub const UNDO: Keycode = Keycode::from_evdev(131);
/// `FRONT = 132`
pub const FRONT: Keycode = Keycode::from_evdev(132);
/// `COPY = 133`
pub const COPY: Keycode = Keycode::from_evdev(133);
/// `OPEN = 134`
pub const OPEN: Keycode = Keycode::from_evdev(134);
/// `PASTE = 135`
pub const PASTE: Keycode = Keycode::from_evdev(135);
/// `FIND = 136`
pub const FIND: Keycode = Keycode::from_evdev(136);
/// `CUT = 137`
pub const CUT: Keycode = Keycode::from_evdev(137);
/// `HELP = 138`
pub const HELP: Keycode = Keycode::from_evdev(138);
/// `MENU = 139`
pub const MENU: Keycode = Keycode::from_evdev(139);
/// `CALC = 140`
pub const CALC: Keycode = Keycode::from_evdev(140);
/// `SETUP = 141`
pub const SETUP: Keycode = Keycode::from_evdev(141);
/// `SLEEP = 142`
pub const SLEEP: Keycode = Keycode::from_evdev(142);
/// `WAKEUP = 143`
pub const WAKEUP: Keycode = Keycode::from_evdev(143);
/// `FILE = 144`
pub const FILE: Keycode = Keycode::from_evdev(144);
/// `SENDFILE = 145`
pub const SENDFILE: Keycode = Keycode::from_evdev(145);
/// `DELETEFILE = 146`
pub const DELETEFILE: Keycode = Keycode::from_evdev(146);
/// `XFER = 147`
pub const XFER: Keycode = Keycode::from_evdev(147);
/// `PROG1 = 148`
pub const PROG1: Keycode = Keycode::from_evdev(148);
/// `PROG2 = 149`
pub const PROG2: Keycode = Keycode::from_evdev(149);
/// `WWW = 150`
pub const WWW: Keycode = Keycode::from_evdev(150);
/// `MSDOS = 151`
pub const MSDOS: Keycode = Keycode::from_evdev(151);
/// `COFFEE = 152`
pub const COFFEE: Keycode = Keycode::from_evdev(152);
/// `SCREENLOCK = 152`
pub const SCREENLOCK: Keycode = Keycode::from_evdev(152);
/// `ROTATE_DISPLAY = 153`
pub const ROTATE_DISPLAY: Keycode = Keycode::from_evdev(153);
/// `DIRECTION = 153`
pub const DIRECTION: Keycode = Keycode::from_evdev(153);
/// `CYCLEWINDOWS = 154`
pub const CYCLEWINDOWS: Keycode = Keycode::from_evdev(154);
/// `MAIL = 155`
pub const MAIL: Keycode = Keycode::from_evdev(155);
/// `BOOKMARKS = 156`
pub const BOOKMARKS: Keycode = Keycode::from_evdev(156);
/// `COMPUTER = 157`
pub const COMPUTER: Keycode = Keycode::from_evdev(157);
/// `BACK = 158`
pub const BACK: Keycode = Keycode::from_evdev(158);
/// `FORWARD = 159`
pub const FORWARD: Keycode = Keycode::from_evdev(159);
/// `CLOSECD = 160`
pub const CLOSECD: Keycode = Keycode::from_evdev(160);
/// `EJECTCD = 161`
pub const EJECTCD: Keycode = Keycode::from_evdev(161);
/// `EJECTCLOSECD = 162`
pub const EJECTCLOSECD: Keycode = Keycode::from_evdev(162);
/// `NEXTSONG = 163`
pub const NEXTSONG: Keycode = Keycode::from_evdev(163);
/// `PLAYPAUSE = 164`
pub const PLAYPAUSE: Keycode = Keycode::from_evdev(164);
/// `PREVIOUSSONG = 165`
pub const PREVIOUSSONG: Keycode = Keycode::from_evdev(165);
/// `STOPCD = 166`
pub const STOPCD: Keycode = Keycode::from_evdev(166);
/// `RECORD = 167`
pub const RECORD: Keycode = Keycode::from_evdev(167);
/// `REWIND = 168`
pub const REWIND: Keycode = Keycode::from_evdev(168);
/// `PHONE = 169`
pub const PHONE: Keycode = Keycode::from_evdev(169);
/// `ISO = 170`
pub const ISO: Keycode = Keycode::from_evdev(170);
/// `CONFIG = 171`
pub const CONFIG: Keycode = Keycode::from_evdev(171);
/// `HOMEPAGE = 172`
pub const HOMEPAGE: Keycode = Keycode::from_evdev(172);
/// `REFRESH = 173`
pub const REFRESH: Keycode = Keycode::from_evdev(173);
/// `EXIT = 174`
pub const EXIT: Keycode = Keycode::from_evdev(174);
/// `MOVE = 175`
pub const MOVE: Keycode = Keycode::from_evdev(175);
/// `EDIT = 176`
pub const EDIT: Keycode = Keycode::from_evdev(176);
/// `SCROLLUP = 177`
pub const SCROLLUP: Keycode = Keycode::from_evdev(177);
/// `SCROLLDOWN = 178`
pub const SCROLLDOWN: Keycode = Keycode::from_evdev(178);
/// `KPLEFTPAREN = 179`
pub const KPLEFTPAREN: Keycode = Keycode::from_evdev(179);
/// `KPRIGHTPAREN = 180`
pub const KPRIGHTPAREN: Keycode = Keycode::from_evdev(180);
/// `NEW = 181`
pub const NEW: Keycode = Keycode::from_evdev(181);
/// `REDO = 182`
pub const REDO: Keycode = Keycode::from_evdev(182);
/// `F13 = 183`
pub const F13: Keycode = Keycode::from_evdev(183);
/// `F14 = 184`
pub const F14: Keycode = Keycode::from_evdev(184);
/// `F15 = 185`
pub const F15: Keycode = Keycode::from_evdev(185);
/// `F16 = 186`
pub const F16: Keycode = Keycode::from_evdev(186);
/// `F17 = 187`
pub const F17: Keycode = Keycode::from_evdev(187);
/// `F18 = 188`
pub const F18: Keycode = Keycode::from_evdev(188);
/// `F19 = 189`
pub const F19: Keycode = Keycode::from_evdev(189);
/// `F20 = 190`
pub const F20: Keycode = Keycode::from_evdev(190);
/// `F21 = 191`
pub const F21: Keycode = Keycode::from_evdev(191);
/// `F22 = 192`
pub const F22: Keycode = Keycode::from_evdev(192);
/// `F23 = 193`
pub const F23: Keycode = Keycode::from_evdev(193);
/// `F24 = 194`
pub const F24: Keycode = Keycode::from_evdev(194);
/// `PLAYCD = 200`
pub const PLAYCD: Keycode = Keycode::from_evdev(200);
/// `PAUSECD = 201`
pub const PAUSECD: Keycode = Keycode::from_evdev(201);
/// `PROG3 = 202`
pub const PROG3: Keycode = Keycode::from_evdev(202);
/// `PROG4 = 203`
pub const PROG4: Keycode = Keycode::from_evdev(203);
/// `ALL_APPLICATIONS = 204`
pub const ALL_APPLICATIONS: Keycode = Keycode::from_evdev(204);
/// `DASHBOARD = 204`
pub const DASHBOARD: Keycode = Keycode::from_evdev(204);
/// `SUSPEND = 205`
pub const SUSPEND: Keycode = Keycode::from_evdev(205);
/// `CLOSE = 206`
pub const CLOSE: Keycode = Keycode::from_evdev(206);
/// `PLAY = 207`
pub const PLAY: Keycode = Keycode::from_evdev(207);
/// `FASTFORWARD = 208`
pub const FASTFORWARD: Keycode = Keycode::from_evdev(208);
/// `BASSBOOST = 209`
pub const BASSBOOST: Keycode = Keycode::from_evdev(209);
/// `PRINT = 210`
pub const PRINT: Keycode = Keycode::from_evdev(210);
/// `HP = 211`
pub const HP: Keycode = Keycode::from_evdev(211);
/// `CAMERA = 212`
pub const CAMERA: Keycode = Keycode::from_evdev(212);
/// `SOUND = 213`
pub const SOUND: Keycode = Keycode::from_evdev(213);
/// `QUESTION = 214`
pub const QUESTION: Keycode = Keycode::from_evdev(214);
/// `EMAIL = 215`
pub const EMAIL: Keycode = Keycode::from_evdev(215);
/// `CHAT = 216`
pub const CHAT: Keycode = Keycode::from_evdev(216);
/// `SEARCH = 217`
pub const SEARCH: Keycode = Keycode::from_evdev(217);
/// `CONNECT = 218`
pub const CONNECT: Keycode = Keycode::from_evdev(218);
/// `FINANCE = 219`
pub const FINANCE: Keycode = Keycode::from_evdev(219);
/// `SPORT = 220`
pub const SPORT: Keycode = Keycode::from_evdev(220);
/// `SHOP = 221`
pub const SHOP: Keycode = Keycode::from_evdev(221);
/// `ALTERASE = 222`
pub const ALTERASE: Keycode = Keycode::from_evdev(222);
/// `CANCEL = 223`
pub const CANCEL: Keycode = Keycode::from_evdev(223);
/// `BRIGHTNESSDOWN = 224`
pub const BRIGHTNESSDOWN: Keycode = Keycode::from_evdev(224);
/// `BRIGHTNESSUP = 225`
pub const BRIGHTNESSUP: Keycode = Keycode::from_evdev(225);
/// `MEDIA = 226`
pub const MEDIA: Keycode = Keycode::from_evdev(226);
/// `SWITCHVIDEOMODE = 227`
pub const SWITCHVIDEOMODE: Keycode = Keycode::from_evdev(227);
/// `KBDILLUMTOGGLE = 228`
pub const KBDILLUMTOGGLE: Keycode = Keycode::from_evdev(228);
/// `KBDILLUMDOWN = 229`
pub const KBDILLUMDOWN: Keycode = Keycode::from_evdev(229);
/// `KBDILLUMUP = 230`
pub const KBDILLUMUP: Keycode = Keycode::from_evdev(230);
/// `SEND = 231`
pub const SEND: Keycode = Keycode::from_evdev(231);
/// `REPLY = 232`
pub const REPLY: Keycode = Keycode::from_evdev(232);
/// `FORWARDMAIL = 233`
pub const FORWARDMAIL: Keycode = Keycode::from_evdev(233);
/// `SAVE = 234`
pub const SAVE: Keycode = Keycode::from_evdev(234);
/// `DOCUMENTS = 235`
pub const DOCUMENTS: Keycode = Keycode::from_evdev(235);
/// `BATTERY = 236`
pub const BATTERY: Keycode = Keycode::from_evdev(236);
/// `BLUETOOTH = 237`
pub const BLUETOOTH: Keycode = Keycode::from_evdev(237);
/// `WLAN = 238`
pub const WLAN: Keycode = Keycode::from_evdev(238);
/// `UWB = 239`
pub const UWB: Keycode = Keycode::from_evdev(239);
/// `UNKNOWN = 240`
pub const UNKNOWN: Keycode = Keycode::from_evdev(240);
/// `VIDEO_NEXT = 241`
pub const VIDEO_NEXT: Keycode = Keycode::from_evdev(241);
/// `VIDEO_PREV = 242`
pub const VIDEO_PREV: Keycode = Keycode::from_evdev(242);
/// `BRIGHTNESS_CYCLE = 243`
pub const BRIGHTNESS_CYCLE: Keycode = Keycode::from_evdev(243);
/// `BRIGHTNESS_AUTO = 244`
pub const BRIGHTNESS_AUTO: Keycode = Keycode::from_evdev(244);
/// `BRIGHTNESS_ZERO = 244`
pub const BRIGHTNESS_ZERO: Keycode = Keycode::from_evdev(244);
/// `DISPLAY_OFF = 245`
pub const DISPLAY_OFF: Keycode = Keycode::from_evdev(245);
/// `WWAN = 246`
pub const WWAN: Keycode = Keycode::from_evdev(246);
/// `WIMAX = 246`
pub const WIMAX: Keycode = Keycode::from_evdev(246);
/// `RFKILL = 247`
pub const RFKILL: Keycode = Keycode::from_evdev(247);
/// `MICMUTE = 248`
pub const MICMUTE: Keycode = Keycode::from_evdev(248);
/// `OK = 352`
pub const OK: Keycode = Keycode::from_evdev(352);
/// `SELECT = 353`
pub const SELECT: Keycode = Keycode::from_evdev(353);
/// `GOTO = 354`
pub const GOTO: Keycode = Keycode::from_evdev(354);
/// `CLEAR = 355`
pub const CLEAR: Keycode = Keycode::from_evdev(355);
/// `POWER2 = 356`
pub const POWER2: Keycode = Keycode::from_evdev(356);
/// `OPTION = 357`
pub const OPTION: Keycode = Keycode::from_evdev(357);
/// `INFO = 358`
pub const INFO: Keycode = Keycode::from_evdev(358);
/// `TIME = 359`
pub const TIME: Keycode = Keycode::from_evdev(359);
/// `VENDOR = 360`
pub const VENDOR: Keycode = Keycode::from_evdev(360);
/// `ARCHIVE = 361`
pub const ARCHIVE: Keycode = Keycode::from_evdev(361);
/// `PROGRAM = 362`
pub const PROGRAM: Keycode = Keycode::from_evdev(362);
/// `CHANNEL = 363`
pub const CHANNEL: Keycode = Keycode::from_evdev(363);
/// `FAVORITES = 364`
pub const FAVORITES: Keycode = Keycode::from_evdev(364);
/// `EPG = 365`
pub const EPG: Keycode = Keycode::from_evdev(365);
/// `PVR = 366`
pub const PVR: Keycode = Keycode::from_evdev(366);
/// `MHP = 367`
pub const MHP: Keycode = Keycode::from_evdev(367);
/// `LANGUAGE = 368`
pub const LANGUAGE: Keycode = Keycode::from_evdev(368);
/// `TITLE = 369`
pub const TITLE: Keycode = Keycode::from_evdev(369);
/// `SUBTITLE = 370`
pub const SUBTITLE: Keycode = Keycode::from_evdev(370);
/// `ANGLE = 371`
pub const ANGLE: Keycode = Keycode::from_evdev(371);
/// `FULL_SCREEN = 372`
pub const FULL_SCREEN: Keycode = Keycode::from_evdev(372);
/// `ZOOM = 372`
pub const ZOOM: Keycode = Keycode::from_evdev(372);
/// `MODE = 373`
pub const MODE: Keycode = Keycode::from_evdev(373);
/// `KEYBOARD = 374`
pub const KEYBOARD: Keycode = Keycode::from_evdev(374);
/// `ASPECT_RATIO = 375`
pub const ASPECT_RATIO: Keycode = Keycode::from_evdev(375);
/// `SCREEN = 375`
pub const SCREEN: Keycode = Keycode::from_evdev(375);
/// `PC = 376`
pub const PC: Keycode = Keycode::from_evdev(376);
/// `TV = 377`
pub const TV: Keycode = Keycode::from_evdev(377);
/// `TV2 = 378`
pub const TV2: Keycode = Keycode::from_evdev(378);
/// `VCR = 379`
pub const VCR: Keycode = Keycode::from_evdev(379);
/// `VCR2 = 380`
pub const VCR2: Keycode = Keycode::from_evdev(380);
/// `SAT = 381`
pub const SAT: Keycode = Keycode::from_evdev(381);
/// `SAT2 = 382`
pub const SAT2: Keycode = Keycode::from_evdev(382);
/// `CD = 383`
pub const CD: Keycode = Keycode::from_evdev(383);
/// `TAPE = 384`
pub const TAPE: Keycode = Keycode::from_evdev(384);
/// `RADIO = 385`
pub const RADIO: Keycode = Keycode::from_evdev(385);
/// `TUNER = 386`
pub const TUNER: Keycode = Keycode::from_evdev(386);
/// `PLAYER = 387`
pub const PLAYER: Keycode = Keycode::from_evdev(387);
/// `TEXT = 388`
pub const TEXT: Keycode = Keycode::from_evdev(388);
/// `DVD = 389`
pub const DVD: Keycode = Keycode::from_evdev(389);
/// `AUX = 390`
pub const AUX: Keycode = Keycode::from_evdev(390);
/// `MP3 = 391`
pub const MP3: Keycode = Keycode::from_evdev(391);
/// `AUDIO = 392`
pub const AUDIO: Keycode = Keycode::from_evdev(392);
/// `VIDEO = 393`
pub const VIDEO: Keycode = Keycode::from_evdev(393);
/// `DIRECTORY = 394`
pub const DIRECTORY: Keycode = Keycode::from_evdev(394);
/// `LIST = 395`
pub const LIST: Keycode = Keycode::from_evdev(395);
/// `MEMO = 396`
pub const MEMO: Keycode = Keycode::from_evdev(396);
/// `CALENDAR = 397`
pub const CALENDAR: Keycode = Keycode::from_evdev(397);
/// `RED = 398`
pub const RED: Keycode = Keycode::from_evdev(398);
/// `GREEN = 399`
pub const GREEN: Keycode = Keycode::from_evdev(399);
/// `YELLOW = 400`
pub const YELLOW: Keycode = Keycode::from_evdev(400);
/// `BLUE = 401`
pub const BLUE: Keycode = Keycode::from_evdev(401);
/// `CHANNELUP = 402`
pub const CHANNELUP: Keycode = Keycode::from_evdev(402);
/// `CHANNELDOWN = 403`
pub const CHANNELDOWN: Keycode = Keycode::from_evdev(403);
/// `FIRST = 404`
pub const FIRST: Keycode = Keycode::from_evdev(404);
/// `LAST = 405`
pub const LAST: Keycode = Keycode::from_evdev(405);
/// `AB = 406`
pub const AB: Keycode = Keycode::from_evdev(406);
/// `NEXT = 407`
pub const NEXT: Keycode = Keycode::from_evdev(407);
/// `RESTART = 408`
pub const RESTART: Keycode = Keycode::from_evdev(408);
/// `SLOW = 409`
pub const SLOW: Keycode = Keycode::from_evdev(409);
/// `SHUFFLE = 410`
pub const SHUFFLE: Keycode = Keycode::from_evdev(410);
/// `BREAK = 411`
pub const BREAK: Keycode = Keycode::from_evdev(411);
/// `PREVIOUS = 412`
pub const PREVIOUS: Keycode = Keycode::from_evdev(412);
/// `DIGITS = 413`
pub const DIGITS: Keycode = Keycode::from_evdev(413);
/// `TEEN = 414`
pub const TEEN: Keycode = Keycode::from_evdev(414);
/// `TWEN = 415`
pub const TWEN: Keycode = Keycode::from_evdev(415);
/// `VIDEOPHONE = 416`
pub const VIDEOPHONE: Keycode = Keycode::from_evdev(416);
/// `GAMES = 417`
pub const GAMES: Keycode = Keycode::from_evdev(417);
/// `ZOOMIN = 418`
pub const ZOOMIN: Keycode = Keycode::from_evdev(418);
/// `ZOOMOUT = 419`
pub const ZOOMOUT: Keycode = Keycode::from_evdev(419);
/// `ZOOMRESET = 420`
pub const ZOOMRESET: Keycode = Keycode::from_evdev(420);
/// `WORDPROCESSOR = 421`
pub const WORDPROCESSOR: Keycode = Keycode::from_evdev(421);
/// `EDITOR = 422`
pub const EDITOR: Keycode = Keycode::from_evdev(422);
/// `SPREADSHEET = 423`
pub const SPREADSHEET: Keycode = Keycode::from_evdev(423);
/// `GRAPHICSEDITOR = 424`
pub const GRAPHICSEDITOR: Keycode = Keycode::from_evdev(424);
/// `PRESENTATION = 425`
pub const PRESENTATION: Keycode = Keycode::from_evdev(425);
/// `DATABASE = 426`
pub const DATABASE: Keycode = Keycode::from_evdev(426);
/// `NEWS = 427`
pub const NEWS: Keycode = Keycode::from_evdev(427);
/// `VOICEMAIL = 428`
pub const VOICEMAIL: Keycode = Keycode::from_evdev(428);
/// `ADDRESSBOOK = 429`
pub const ADDRESSBOOK: Keycode = Keycode::from_evdev(429);
/// `MESSENGER = 430`
pub const MESSENGER: Keycode = Keycode::from_evdev(430);
/// `DISPLAYTOGGLE = 431`
pub const DISPLAYTOGGLE: Keycode = Keycode::from_evdev(431);
/// `BRIGHTNESS_TOGGLE = 431`
pub const BRIGHTNESS_TOGGLE: Keycode = Keycode::from_evdev(431);
/// `SPELLCHECK = 432`
pub const SPELLCHECK: Keycode = Keycode::from_evdev(432);
/// `LOGOFF = 433`
pub const LOGOFF: Keycode = Keycode::from_evdev(433);
/// `DOLLAR = 434`
pub const DOLLAR: Keycode = Keycode::from_evdev(434);
/// `EURO = 435`
pub const EURO: Keycode = Keycode::from_evdev(435);
/// `FRAMEBACK = 436`
pub const FRAMEBACK: Keycode = Keycode::from_evdev(436);
/// `FRAMEFORWARD = 437`
pub const FRAMEFORWARD: Keycode = Keycode::from_evdev(437);
/// `CONTEXT_MENU = 438`
pub const CONTEXT_MENU: Keycode = Keycode::from_evdev(438);
/// `MEDIA_REPEAT = 439`
pub const MEDIA_REPEAT: Keycode = Keycode::from_evdev(439);
/// `10CHANNELSUP = 440`
pub const _10CHANNELSUP: Keycode = Keycode::from_evdev(440);
/// `10CHANNELSDOWN = 441`
pub const _10CHANNELSDOWN: Keycode = Keycode::from_evdev(441);
/// `IMAGES = 442`
pub const IMAGES: Keycode = Keycode::from_evdev(442);
/// `NOTIFICATION_CENTER = 444`
pub const NOTIFICATION_CENTER: Keycode = Keycode::from_evdev(444);
/// `PICKUP_PHONE = 445`
pub const PICKUP_PHONE: Keycode = Keycode::from_evdev(445);
/// `HANGUP_PHONE = 446`
pub const HANGUP_PHONE: Keycode = Keycode::from_evdev(446);
/// `DEL_EOL = 448`
pub const DEL_EOL: Keycode = Keycode::from_evdev(448);
/// `DEL_EOS = 449`
pub const DEL_EOS: Keycode = Keycode::from_evdev(449);
/// `INS_LINE = 450`
pub const INS_LINE: Keycode = Keycode::from_evdev(450);
/// `DEL_LINE = 451`
pub const DEL_LINE: Keycode = Keycode::from_evdev(451);
/// `FN = 464`
pub const FN: Keycode = Keycode::from_evdev(464);
/// `FN_ESC = 465`
pub const FN_ESC: Keycode = Keycode::from_evdev(465);
/// `FN_F1 = 466`
pub const FN_F1: Keycode = Keycode::from_evdev(466);
/// `FN_F2 = 467`
pub const FN_F2: Keycode = Keycode::from_evdev(467);
/// `FN_F3 = 468`
pub const FN_F3: Keycode = Keycode::from_evdev(468);
/// `FN_F4 = 469`
pub const FN_F4: Keycode = Keycode::from_evdev(469);
/// `FN_F5 = 470`
pub const FN_F5: Keycode = Keycode::from_evdev(470);
/// `FN_F6 = 471`
pub const FN_F6: Keycode = Keycode::from_evdev(471);
/// `FN_F7 = 472`
pub const FN_F7: Keycode = Keycode::from_evdev(472);
/// `FN_F8 = 473`
pub const FN_F8: Keycode = Keycode::from_evdev(473);
/// `FN_F9 = 474`
pub const FN_F9: Keycode = Keycode::from_evdev(474);
/// `FN_F10 = 475`
pub const FN_F10: Keycode = Keycode::from_evdev(475);
/// `FN_F11 = 476`
pub const FN_F11: Keycode = Keycode::from_evdev(476);
/// `FN_F12 = 477`
pub const FN_F12: Keycode = Keycode::from_evdev(477);
/// `FN_1 = 478`
pub const FN_1: Keycode = Keycode::from_evdev(478);
/// `FN_2 = 479`
pub const FN_2: Keycode = Keycode::from_evdev(479);
/// `FN_D = 480`
pub const FN_D: Keycode = Keycode::from_evdev(480);
/// `FN_E = 481`
pub const FN_E: Keycode = Keycode::from_evdev(481);
/// `FN_F = 482`
pub const FN_F: Keycode = Keycode::from_evdev(482);
/// `FN_S = 483`
pub const FN_S: Keycode = Keycode::from_evdev(483);
/// `FN_B = 484`
pub const FN_B: Keycode = Keycode::from_evdev(484);
/// `FN_RIGHT_SHIFT = 485`
pub const FN_RIGHT_SHIFT: Keycode = Keycode::from_evdev(485);
/// `BRL_DOT1 = 497`
pub const BRL_DOT1: Keycode = Keycode::from_evdev(497);
/// `BRL_DOT2 = 498`
pub const BRL_DOT2: Keycode = Keycode::from_evdev(498);
/// `BRL_DOT3 = 499`
pub const BRL_DOT3: Keycode = Keycode::from_evdev(499);
/// `BRL_DOT4 = 500`
pub const BRL_DOT4: Keycode = Keycode::from_evdev(500);
/// `BRL_DOT5 = 501`
pub const BRL_DOT5: Keycode = Keycode::from_evdev(501);
/// `BRL_DOT6 = 502`
pub const BRL_DOT6: Keycode = Keycode::from_evdev(502);
/// `BRL_DOT7 = 503`
pub const BRL_DOT7: Keycode = Keycode::from_evdev(503);
/// `BRL_DOT8 = 504`
pub const BRL_DOT8: Keycode = Keycode::from_evdev(504);
/// `BRL_DOT9 = 505`
pub const BRL_DOT9: Keycode = Keycode::from_evdev(505);
/// `BRL_DOT10 = 506`
pub const BRL_DOT10: Keycode = Keycode::from_evdev(506);
/// `NUMERIC_0 = 512`
pub const NUMERIC_0: Keycode = Keycode::from_evdev(512);
/// `NUMERIC_1 = 513`
pub const NUMERIC_1: Keycode = Keycode::from_evdev(513);
/// `NUMERIC_2 = 514`
pub const NUMERIC_2: Keycode = Keycode::from_evdev(514);
/// `NUMERIC_3 = 515`
pub const NUMERIC_3: Keycode = Keycode::from_evdev(515);
/// `NUMERIC_4 = 516`
pub const NUMERIC_4: Keycode = Keycode::from_evdev(516);
/// `NUMERIC_5 = 517`
pub const NUMERIC_5: Keycode = Keycode::from_evdev(517);
/// `NUMERIC_6 = 518`
pub const NUMERIC_6: Keycode = Keycode::from_evdev(518);
/// `NUMERIC_7 = 519`
pub const NUMERIC_7: Keycode = Keycode::from_evdev(519);
/// `NUMERIC_8 = 520`
pub const NUMERIC_8: Keycode = Keycode::from_evdev(520);
/// `NUMERIC_9 = 521`
pub const NUMERIC_9: Keycode = Keycode::from_evdev(521);
/// `NUMERIC_STAR = 522`
pub const NUMERIC_STAR: Keycode = Keycode::from_evdev(522);
/// `NUMERIC_POUND = 523`
pub const NUMERIC_POUND: Keycode = Keycode::from_evdev(523);
/// `NUMERIC_A = 524`
pub const NUMERIC_A: Keycode = Keycode::from_evdev(524);
/// `NUMERIC_B = 525`
pub const NUMERIC_B: Keycode = Keycode::from_evdev(525);
/// `NUMERIC_C = 526`
pub const NUMERIC_C: Keycode = Keycode::from_evdev(526);
/// `NUMERIC_D = 527`
pub const NUMERIC_D: Keycode = Keycode::from_evdev(527);
/// `CAMERA_FOCUS = 528`
pub const CAMERA_FOCUS: Keycode = Keycode::from_evdev(528);
/// `WPS_BUTTON = 529`
pub const WPS_BUTTON: Keycode = Keycode::from_evdev(529);
/// `TOUCHPAD_TOGGLE = 530`
pub const TOUCHPAD_TOGGLE: Keycode = Keycode::from_evdev(530);
/// `TOUCHPAD_ON = 531`
pub const TOUCHPAD_ON: Keycode = Keycode::from_evdev(531);
/// `TOUCHPAD_OFF = 532`
pub const TOUCHPAD_OFF: Keycode = Keycode::from_evdev(532);
/// `CAMERA_ZOOMIN = 533`
pub const CAMERA_ZOOMIN: Keycode = Keycode::from_evdev(533);
/// `CAMERA_ZOOMOUT = 534`
pub const CAMERA_ZOOMOUT: Keycode = Keycode::from_evdev(534);
/// `CAMERA_UP = 535`
pub const CAMERA_UP: Keycode = Keycode::from_evdev(535);
/// `CAMERA_DOWN = 536`
pub const CAMERA_DOWN: Keycode = Keycode::from_evdev(536);
/// `CAMERA_LEFT = 537`
pub const CAMERA_LEFT: Keycode = Keycode::from_evdev(537);
/// `CAMERA_RIGHT = 538`
pub const CAMERA_RIGHT: Keycode = Keycode::from_evdev(538);
/// `ATTENDANT_ON = 539`
pub const ATTENDANT_ON: Keycode = Keycode::from_evdev(539);
/// `ATTENDANT_OFF = 540`
pub const ATTENDANT_OFF: Keycode = Keycode::from_evdev(540);
/// `ATTENDANT_TOGGLE = 541`
pub const ATTENDANT_TOGGLE: Keycode = Keycode::from_evdev(541);
/// `LIGHTS_TOGGLE = 542`
pub const LIGHTS_TOGGLE: Keycode = Keycode::from_evdev(542);
/// `ALS_TOGGLE = 560`
pub const ALS_TOGGLE: Keycode = Keycode::from_evdev(560);
/// `ROTATE_LOCK_TOGGLE = 561`
pub const ROTATE_LOCK_TOGGLE: Keycode = Keycode::from_evdev(561);
/// `REFRESH_RATE_TOGGLE = 562`
pub const REFRESH_RATE_TOGGLE: Keycode = Keycode::from_evdev(562);
/// `BUTTONCONFIG = 576`
pub const BUTTONCONFIG: Keycode = Keycode::from_evdev(576);
/// `TASKMANAGER = 577`
pub const TASKMANAGER: Keycode = Keycode::from_evdev(577);
/// `JOURNAL = 578`
pub const JOURNAL: Keycode = Keycode::from_evdev(578);
/// `CONTROLPANEL = 579`
pub const CONTROLPANEL: Keycode = Keycode::from_evdev(579);
/// `APPSELECT = 580`
pub const APPSELECT: Keycode = Keycode::from_evdev(580);
/// `SCREENSAVER = 581`
pub const SCREENSAVER: Keycode = Keycode::from_evdev(581);
/// `VOICECOMMAND = 582`
pub const VOICECOMMAND: Keycode = Keycode::from_evdev(582);
/// `ASSISTANT = 583`
pub const ASSISTANT: Keycode = Keycode::from_evdev(583);
/// `KBD_LAYOUT_NEXT = 584`
pub const KBD_LAYOUT_NEXT: Keycode = Keycode::from_evdev(584);
/// `EMOJI_PICKER = 585`
pub const EMOJI_PICKER: Keycode = Keycode::from_evdev(585);
/// `DICTATE = 586`
pub const DICTATE: Keycode = Keycode::from_evdev(586);
/// `CAMERA_ACCESS_ENABLE = 587`
pub const CAMERA_ACCESS_ENABLE: Keycode = Keycode::from_evdev(587);
/// `CAMERA_ACCESS_DISABLE = 588`
pub const CAMERA_ACCESS_DISABLE: Keycode = Keycode::from_evdev(588);
/// `CAMERA_ACCESS_TOGGLE = 589`
pub const CAMERA_ACCESS_TOGGLE: Keycode = Keycode::from_evdev(589);
/// `ACCESSIBILITY = 590`
pub const ACCESSIBILITY: Keycode = Keycode::from_evdev(590);
/// `DO_NOT_DISTURB = 591`
pub const DO_NOT_DISTURB: Keycode = Keycode::from_evdev(591);
/// `BRIGHTNESS_MIN = 592`
pub const BRIGHTNESS_MIN: Keycode = Keycode::from_evdev(592);
/// `BRIGHTNESS_MAX = 593`
pub const BRIGHTNESS_MAX: Keycode = Keycode::from_evdev(593);
/// `KBDINPUTASSIST_PREV = 608`
pub const KBDINPUTASSIST_PREV: Keycode = Keycode::from_evdev(608);
/// `KBDINPUTASSIST_NEXT = 609`
pub const KBDINPUTASSIST_NEXT: Keycode = Keycode::from_evdev(609);
/// `KBDINPUTASSIST_PREVGROUP = 610`
pub const KBDINPUTASSIST_PREVGROUP: Keycode = Keycode::from_evdev(610);
/// `KBDINPUTASSIST_NEXTGROUP = 611`
pub const KBDINPUTASSIST_NEXTGROUP: Keycode = Keycode::from_evdev(611);
/// `KBDINPUTASSIST_ACCEPT = 612`
pub const KBDINPUTASSIST_ACCEPT: Keycode = Keycode::from_evdev(612);
/// `KBDINPUTASSIST_CANCEL = 613`
pub const KBDINPUTASSIST_CANCEL: Keycode = Keycode::from_evdev(613);
/// `RIGHT_UP = 614`
pub const RIGHT_UP: Keycode = Keycode::from_evdev(614);
/// `RIGHT_DOWN = 615`
pub const RIGHT_DOWN: Keycode = Keycode::from_evdev(615);
/// `LEFT_UP = 616`
pub const LEFT_UP: Keycode = Keycode::from_evdev(616);
/// `LEFT_DOWN = 617`
pub const LEFT_DOWN: Keycode = Keycode::from_evdev(617);
/// `ROOT_MENU = 618`
pub const ROOT_MENU: Keycode = Keycode::from_evdev(618);
/// `MEDIA_TOP_MENU = 619`
pub const MEDIA_TOP_MENU: Keycode = Keycode::from_evdev(619);
/// `NUMERIC_11 = 620`
pub const NUMERIC_11: Keycode = Keycode::from_evdev(620);
/// `NUMERIC_12 = 621`
pub const NUMERIC_12: Keycode = Keycode::from_evdev(621);
/// `AUDIO_DESC = 622`
pub const AUDIO_DESC: Keycode = Keycode::from_evdev(622);
/// `3D_MODE = 623`
pub const _3D_MODE: Keycode = Keycode::from_evdev(623);
/// `NEXT_FAVORITE = 624`
pub const NEXT_FAVORITE: Keycode = Keycode::from_evdev(624);
/// `STOP_RECORD = 625`
pub const STOP_RECORD: Keycode = Keycode::from_evdev(625);
/// `PAUSE_RECORD = 626`
pub const PAUSE_RECORD: Keycode = Keycode::from_evdev(626);
/// `VOD = 627`
pub const VOD: Keycode = Keycode::from_evdev(627);
/// `UNMUTE = 628`
pub const UNMUTE: Keycode = Keycode::from_evdev(628);
/// `FASTREVERSE = 629`
pub const FASTREVERSE: Keycode = Keycode::from_evdev(629);
/// `SLOWREVERSE = 630`
pub const SLOWREVERSE: Keycode = Keycode::from_evdev(630);
/// `DATA = 631`
pub const DATA: Keycode = Keycode::from_evdev(631);
/// `ONSCREEN_KEYBOARD = 632`
pub const ONSCREEN_KEYBOARD: Keycode = Keycode::from_evdev(632);
/// `PRIVACY_SCREEN_TOGGLE = 633`
pub const PRIVACY_SCREEN_TOGGLE: Keycode = Keycode::from_evdev(633);
/// `SELECTIVE_SCREENSHOT = 634`
pub const SELECTIVE_SCREENSHOT: Keycode = Keycode::from_evdev(634);
/// `NEXT_ELEMENT = 635`
pub const NEXT_ELEMENT: Keycode = Keycode::from_evdev(635);
/// `PREVIOUS_ELEMENT = 636`
pub const PREVIOUS_ELEMENT: Keycode = Keycode::from_evdev(636);
/// `AUTOPILOT_ENGAGE_TOGGLE = 637`
pub const AUTOPILOT_ENGAGE_TOGGLE: Keycode = Keycode::from_evdev(637);
/// `MARK_WAYPOINT = 638`
pub const MARK_WAYPOINT: Keycode = Keycode::from_evdev(638);
/// `SOS = 639`
pub const SOS: Keycode = Keycode::from_evdev(639);
/// `NAV_CHART = 640`
pub const NAV_CHART: Keycode = Keycode::from_evdev(640);
/// `FISHING_CHART = 641`
pub const FISHING_CHART: Keycode = Keycode::from_evdev(641);
/// `SINGLE_RANGE_RADAR = 642`
pub const SINGLE_RANGE_RADAR: Keycode = Keycode::from_evdev(642);
/// `DUAL_RANGE_RADAR = 643`
pub const DUAL_RANGE_RADAR: Keycode = Keycode::from_evdev(643);
/// `RADAR_OVERLAY = 644`
pub const RADAR_OVERLAY: Keycode = Keycode::from_evdev(644);
/// `TRADITIONAL_SONAR = 645`
pub const TRADITIONAL_SONAR: Keycode = Keycode::from_evdev(645);
/// `CLEARVU_SONAR = 646`
pub const CLEARVU_SONAR: Keycode = Keycode::from_evdev(646);
/// `SIDEVU_SONAR = 647`
pub const SIDEVU_SONAR: Keycode = Keycode::from_evdev(647);
/// `NAV_INFO = 648`
pub const NAV_INFO: Keycode = Keycode::from_evdev(648);
/// `BRIGHTNESS_MENU = 649`
pub const BRIGHTNESS_MENU: Keycode = Keycode::from_evdev(649);
/// `MACRO1 = 656`
pub const MACRO1: Keycode = Keycode::from_evdev(656);
/// `MACRO2 = 657`
pub const MACRO2: Keycode = Keycode::from_evdev(657);
/// `MACRO3 = 658`
pub const MACRO3: Keycode = Keycode::from_evdev(658);
/// `MACRO4 = 659`
pub const MACRO4: Keycode = Keycode::from_evdev(659);
/// `MACRO5 = 660`
pub const MACRO5: Keycode = Keycode::from_evdev(660);
/// `MACRO6 = 661`
pub const MACRO6: Keycode = Keycode::from_evdev(661);
/// `MACRO7 = 662`
pub const MACRO7: Keycode = Keycode::from_evdev(662);
/// `MACRO8 = 663`
pub const MACRO8: Keycode = Keycode::from_evdev(663);
/// `MACRO9 = 664`
pub const MACRO9: Keycode = Keycode::from_evdev(664);
/// `MACRO10 = 665`
pub const MACRO10: Keycode = Keycode::from_evdev(665);
/// `MACRO11 = 666`
pub const MACRO11: Keycode = Keycode::from_evdev(666);
/// `MACRO12 = 667`
pub const MACRO12: Keycode = Keycode::from_evdev(667);
/// `MACRO13 = 668`
pub const MACRO13: Keycode = Keycode::from_evdev(668);
/// `MACRO14 = 669`
pub const MACRO14: Keycode = Keycode::from_evdev(669);
/// `MACRO15 = 670`
pub const MACRO15: Keycode = Keycode::from_evdev(670);
/// `MACRO16 = 671`
pub const MACRO16: Keycode = Keycode::from_evdev(671);
/// `MACRO17 = 672`
pub const MACRO17: Keycode = Keycode::from_evdev(672);
/// `MACRO18 = 673`
pub const MACRO18: Keycode = Keycode::from_evdev(673);
/// `MACRO19 = 674`
pub const MACRO19: Keycode = Keycode::from_evdev(674);
/// `MACRO20 = 675`
pub const MACRO20: Keycode = Keycode::from_evdev(675);
/// `MACRO21 = 676`
pub const MACRO21: Keycode = Keycode::from_evdev(676);
/// `MACRO22 = 677`
pub const MACRO22: Keycode = Keycode::from_evdev(677);
/// `MACRO23 = 678`
pub const MACRO23: Keycode = Keycode::from_evdev(678);
/// `MACRO24 = 679`
pub const MACRO24: Keycode = Keycode::from_evdev(679);
/// `MACRO25 = 680`
pub const MACRO25: Keycode = Keycode::from_evdev(680);
/// `MACRO26 = 681`
pub const MACRO26: Keycode = Keycode::from_evdev(681);
/// `MACRO27 = 682`
pub const MACRO27: Keycode = Keycode::from_evdev(682);
/// `MACRO28 = 683`
pub const MACRO28: Keycode = Keycode::from_evdev(683);
/// `MACRO29 = 684`
pub const MACRO29: Keycode = Keycode::from_evdev(684);
/// `MACRO30 = 685`
pub const MACRO30: Keycode = Keycode::from_evdev(685);
/// `MACRO_RECORD_START = 688`
pub const MACRO_RECORD_START: Keycode = Keycode::from_evdev(688);
/// `MACRO_RECORD_STOP = 689`
pub const MACRO_RECORD_STOP: Keycode = Keycode::from_evdev(689);
/// `MACRO_PRESET_CYCLE = 690`
pub const MACRO_PRESET_CYCLE: Keycode = Keycode::from_evdev(690);
/// `MACRO_PRESET1 = 691`
pub const MACRO_PRESET1: Keycode = Keycode::from_evdev(691);
/// `MACRO_PRESET2 = 692`
pub const MACRO_PRESET2: Keycode = Keycode::from_evdev(692);
/// `MACRO_PRESET3 = 693`
pub const MACRO_PRESET3: Keycode = Keycode::from_evdev(693);
/// `KBD_LCD_MENU1 = 696`
pub const KBD_LCD_MENU1: Keycode = Keycode::from_evdev(696);
/// `KBD_LCD_MENU2 = 697`
pub const KBD_LCD_MENU2: Keycode = Keycode::from_evdev(697);
/// `KBD_LCD_MENU3 = 698`
pub const KBD_LCD_MENU3: Keycode = Keycode::from_evdev(698);
/// `KBD_LCD_MENU4 = 699`
pub const KBD_LCD_MENU4: Keycode = Keycode::from_evdev(699);
/// `KBD_LCD_MENU5 = 700`
pub const KBD_LCD_MENU5: Keycode = Keycode::from_evdev(700);
