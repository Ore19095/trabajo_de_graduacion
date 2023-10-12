import numpy as np

f = 15625 # Hz, pwm frequency
f_signal = 10 # Hz, signal frequency
N = 1000 # number of period samples
peridos = 20 # number of periods

pwl_text = '0 0\n'

pwl_line_template = '+{t1} {v1}\n+{t2} {v1}\n+{t3} {v2}\n+{t4} {v2}\n'


arch = open('pwl_file.txt', 'w')

for i in range(N*peridos):
    arch.write(pwl_text)

    t = i/f # time
    signal = 0.5*np.sin(2*np.pi*f_signal*t) + 0.5 # signal

    t_on = signal/f # time on
    t_off = (1-signal)/f # time off

    pwl_text = pwl_line_template.format(
        t1 = 1e-12,
        t2 = t_on,
        t3 = 1e-6,
        t4 = t_off,
        v1 = 5,
        v2 = 0
    )

arch.close()
print('Done')    


