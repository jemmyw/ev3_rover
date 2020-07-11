extern crate ev3dev_lang_rust;

use ev3dev_lang_rust::motors::{LargeMotor, MediumMotor, MotorPort};
use ev3dev_lang_rust::sensors::{ColorSensor, InfraredSensor, TouchSensor};
use ev3dev_lang_rust::sound;
use ev3dev_lang_rust::Ev3Result;

// Distance = 4
// Color = 3
// Touch = 2
// Left motor = B
// Right motor = C
// Turn motor = A

enum States {
    Searching,
    Found,
    Avoid,
}

static MAX_COL: i32 = 1020;
static FORWARD_SPEED: i32 = -50;
static BACKWARD_SPEED: i32 = 50;

fn on_color(sen: &ColorSensor) -> bool {
    let val = sen.get_rgb();

    let v: bool = match val {
        Ok((r, g, b)) => r < MAX_COL / 2 && g > MAX_COL / 2 && b > MAX_COL / 2,
        Err(_) => false,
    };

    return v;
}

fn get_distance(sen: &InfraredSensor) -> i32 {
    let val = sen.get_value0();

    return match val {
        Ok(v) => v,
        Err(_) => panic!("Invalid sensor value"),
    };
}

fn get_touch(sen: &TouchSensor) -> bool {
    let val = sen.get_pressed_state();

    return match val {
        Ok(true) => true,
        Ok(false) => false,
        Err(_) => panic!("Invalid touch value"),
    };
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

    let mut state;

    loop {
        let is_on_color = on_color(&color_sensor);
        let distance = get_distance(&ir_sensor);
        let touched = get_touch(&touch_sensor);

        if is_on_color {
            state = States::Found;
        } else if distance < 25 || touched {
            state = States::Avoid;
        } else {
            state = States::Searching;
        }

        match state {
            States::Searching => {
                left_motor.set_duty_cycle_sp(FORWARD_SPEED)?;
                right_motor.set_duty_cycle_sp(FORWARD_SPEED)?;
            }
            States::Avoid => {
                let pos = turn_motor.get_position()?;
                turn_motor.set_position(pos + 10)?;
            }
            States::Found => {
                sound::play("bark.wav")?.wait()?;
                break;
            }
        }
    }

    Ok(())
}
