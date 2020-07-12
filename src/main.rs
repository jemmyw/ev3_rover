extern crate ev3dev_lang_rust;
#[macro_use]
extern crate machine;

use ev3dev_lang_rust::motors::{LargeMotor, MediumMotor, MotorPort};
use ev3dev_lang_rust::sensors::{ColorSensor, InfraredSensor, TouchSensor};
use ev3dev_lang_rust::sound;
use ev3dev_lang_rust::Ev3Result;
use std::thread;
use std::time::Duration;

// Ports for sensors and motors:
// Distance = 4
// Color = 3
// Touch = 2
// Left motor = B
// Right motor = C
// Turn motor = A

// Largest color value
static MAX_COL: i32 = 1020;
static FORWARD_SPEED: i32 = -50;
static BACKWARD_SPEED: i32 = 50;
// Tick length
static TIMEOUT: Duration = Duration::from_millis(250);

/**
 * The world as we know it
 */
#[derive(Clone, Debug, PartialEq)]
pub struct World {
    // Ticks since program started
    tick: u32,
    // Current color in rgb
    color: (i32, i32, i32),
    // Current distance from IR sensor
    distance: i32,
    // Touch sensor pressed
    touched: bool,
    // Small motor turn position
    turn: i32,
}

/**
 * Next device state
 */
#[derive(Clone, Debug, PartialEq)]
pub struct Device {
    // Left motor speed
    left: i32,
    // Right motor speed
    right: i32,
    // Small motor should turn by this much, accepts negative values
    turn: i32,
    // Play a sound
    sound: Option<String>,
}

machine!(
    /**
     * State machine setup.
     *
     * Start -> Searching
     *   -> Found
     *   -> AvoidingBack -> AvoidingTurn -> AvoidingAdvance -> Searching
     */
    enum State {
        Start,
        Searching { world: World },
        Found,
        AvoidingBack { tick: u32, world: World },
        AvoidingTurn { tick: u32, world: World },
        AvoidingAdvance { tick: u32, world: World },
    }
);

transitions!(State,
[
    (Start, World) => Searching,
    (Searching, World) => [Found, AvoidingBack],
    (AvoidingBack, World) => [AvoidingBack, AvoidingTurn],
    (AvoidingTurn, World) => [AvoidingTurn, AvoidingAdvance],
    (AvoidingAdvance, World) => [AvoidingAdvance, Searching]
]);

methods!(State,
[
    Searching, AvoidingBack, AvoidingTurn, AvoidingAdvance, Found => fn process(&self) -> Device
]);

fn is_yellow(c: (i32, i32, i32)) -> bool {
    let (r, g, b) = c;
    r < MAX_COL / 2 && g > MAX_COL / 2 && b > MAX_COL / 2
}

/**
 * Start state just moves straight to Searching
 */
impl Start {
    pub fn on_world(self, world: World) -> Searching {
        Searching { world: world }
    }
}

/**
 * Searching state sets the motors forward. If it sees the colour yellow it
 * goes to Found. If it detects an object ahead via distance or press then it
 * goes to AvoidingBack
 */
impl Searching {
    pub fn on_world(self, world: World) -> State {
        if is_yellow(world.color) {
            return State::found();
        }

        if world.distance < 25 || world.touched {
            return State::avoidingback(0, world);
        }

        State::searching(world)
    }

    pub fn process(&self) -> Device {
        Device {
            left: FORWARD_SPEED,
            right: FORWARD_SPEED,
            turn: 0,
            sound: None,
        }
    }
}

/**
 * For 10 ticks reverse
 */
impl AvoidingBack {
    pub fn on_world(self, world: World) -> State {
        if self.tick < 10 {
            State::avoidingback(self.tick + 1, world)
        } else {
            State::avoidingturn(0, world)
        }
    }

    pub fn process(&self) -> Device {
        Device {
            left: BACKWARD_SPEED,
            right: BACKWARD_SPEED,
            turn: 0,
            sound: None,
        }
    }
}

/**
 * For 1 tick turn the wheels
 */
impl AvoidingTurn {
    pub fn on_world(self, world: World) -> State {
        State::avoidingadvance(0, world)
    }

    pub fn process(&self) -> Device {
        Device {
            left: 0,
            right: 0,
            turn: 10,
            sound: None,
        }
    }
}

/**
 * For 9 ticks advance (with wheels turned) then turn them back in 1 tick
 */
impl AvoidingAdvance {
    fn last_tick(&self) -> bool {
        self.tick == 9
    }

    pub fn on_world(self, world: World) -> State {
        if self.tick < 10 {
            State::avoidingadvance(self.tick + 1, world)
        } else {
            State::searching(world)
        }
    }

    pub fn process(&self) -> Device {
        if self.last_tick() {
            Device {
                left: 0,
                right: 0,
                turn: -10,
                sound: None,
            }
        } else {
            Device {
                left: FORWARD_SPEED / 2,
                right: FORWARD_SPEED / 2,
                turn: 0,
                sound: None,
            }
        }
    }
}

/**
 * When in found state just play a sound
 */
impl Found {
    pub fn process(&self) -> Device {
        Device {
            left: 0,
            right: 0,
            turn: 0,
            sound: Some("bark.wav".to_string()),
        }
    }
}

fn main() -> Ev3Result<()> {
    let left_motor = LargeMotor::get(MotorPort::OutB)?;
    let right_motor = LargeMotor::get(MotorPort::OutC)?;
    let turn_motor = MediumMotor::get(MotorPort::OutA)?;

    // Set command "run-direct".
    left_motor.run_direct()?;
    right_motor.run_direct()?;

    let ir_sensor = InfraredSensor::find()?;
    let color_sensor = ColorSensor::find()?;
    let touch_sensor = TouchSensor::find()?;

    color_sensor.set_mode_rgb_raw()?;
    ir_sensor.set_mode_ir_prox()?;

    let mut state = State::Start(Start {});
    let mut tick = 0;

    loop {
        // Get the world from the sensors
        let world = World {
            tick: tick,
            color: color_sensor.get_rgb()?,
            distance: ir_sensor.get_value0()?,
            touched: touch_sensor.get_pressed_state()?,
            turn: turn_motor.get_position()?,
        };
        // Get the next state
        state = state.on_world(world);
        tick += 1;

        // Get the state's device update
        let device_result = state.process();

        match device_result {
            Some(device) => {
                left_motor.set_duty_cycle_sp(device.left)?;
                right_motor.set_duty_cycle_sp(device.right)?;

                let pos = turn_motor.get_position()?;
                turn_motor.set_position(pos + device.turn)?;

                match device.sound {
                    Some(file) => {
                        sound::play(&file)?.wait()?;
                        break;
                    }
                    None => {}
                }
            }
            None => {}
        }

        thread::sleep(TIMEOUT);
    }

    Ok(())
}
