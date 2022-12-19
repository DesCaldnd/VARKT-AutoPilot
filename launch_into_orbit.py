import math
import time
import krpc

target_altitude = 220000

# Подключаемся к ракете.
conn = krpc.connect(name='Launch into orbit')
vessel = conn.space_center.active_vessel

# Настройка потоков для сбора информации.
ut = conn.add_stream(getattr, conn.space_center, 'ut')
altitude = conn.add_stream(getattr, vessel.flight(), 'mean_altitude')
apoapsis = conn.add_stream(getattr, vessel.orbit, 'apoapsis_altitude')
first_fuel = conn.add_stream(vessel.resources.amount, 'LiquidFuel')

# Выключаем САС и РСУ; выставляем максимальную тягу.
vessel.control.sas = False
vessel.control.rcs = False
vessel.control.throttle = 1.0

print('3...')
time.sleep(1)
print('2...')
time.sleep(1)
print('1...')
time.sleep(1)
print('Launch!')

# Активируем первую ступень; вертикальный полёт.
vessel.control.activate_next_stage()
vessel.auto_pilot.engage()
vessel.auto_pilot.target_pitch_and_heading(90, 90)

# Главный цикл подъёма.
first_separated = False
turn_start_altitude = 250
turn_end_altitude = 45000
turn_angle = 0
while True:
    # Гравитационный поворот при подъёме.
    if altitude() > turn_start_altitude and altitude() < turn_end_altitude:
        frac = ((altitude() - turn_start_altitude) /
                (turn_end_altitude - turn_start_altitude))
        new_turn_angle = frac * 90
        if abs(new_turn_angle - turn_angle) > 0.5:
            turn_angle = new_turn_angle
            vessel.auto_pilot.target_pitch_and_heading(90-turn_angle, 90)

    # Сброс первой ступени без топлива.
    if not first_separated:
        if first_fuel() - 720 < 0.1:
            vessel.control.activate_next_stage() # Сброс первой ступени.
            vessel.control.activate_next_stage() # Активируем вторую ступень.
            first_separated = True
            print('The first stage separated')

    # Уменьшаем тягу до целевого апоцентра.
    if apoapsis() > target_altitude*0.9:
        print('Approaching target apoapsis')
        break

# Отключаем двигатели в целевом апоцентре.
vessel.control.throttle = 0.25
while apoapsis() < target_altitude:
    pass
print('Target apoapsis reached')
vessel.control.throttle = 0.0

# Проверяем выход ракеты из атмосферы.
print('Coasting out of atmosphere')
while altitude() < 70500:
    pass

# Используем уравнение кинетической энергии для нахождения точки и направления манёвра.
print('Planning circularization burn')
mu = vessel.orbit.body.gravitational_parameter
a1 = vessel.orbit.semi_major_axis
a2 = vessel.orbit.apoapsis
delta_v = math.sqrt(mu*((2./a2)-(1./a2))) - math.sqrt(mu*((2./a2)-(1./a1)))
node = vessel.control.add_node(ut() + vessel.orbit.time_to_apoapsis, prograde=delta_v)

# Используем формулу Циолковского и находим время работы двигателей для манёвра.
F = vessel.available_thrust
Isp = vessel.specific_impulse * 9.82
m0 = vessel.mass
m1 = m0 / math.exp(delta_v/Isp)
flow_rate = F / Isp
burn_time = (m0 - m1) / flow_rate

# Поворачиваем ракету по направлению манёвра.
print('Orientating ship for circularization burn')
vessel.auto_pilot.reference_frame = node.reference_frame
vessel.auto_pilot.target_direction = (0, 1, 0)
vessel.auto_pilot.wait()

# Ждём манёвр.
print('Waiting until circularization burn')
burn_ut = ut() + vessel.orbit.time_to_apoapsis - (burn_time/2.)
lead_time = 5
conn.space_center.warp_to(burn_ut - lead_time)

# Выполняем манёвр.
print('Ready to execute burn')
time_to_apoapsis = conn.add_stream(getattr, vessel.orbit, 'time_to_apoapsis')
while time_to_apoapsis() - (burn_time/2.) > 0:
    pass
print('Executing burn')
vessel.control.throttle = 1.0
time.sleep(burn_time - 0.1)
vessel.control.throttle = 0.0
node.remove()
print('Launch complete')
