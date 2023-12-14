import json
import plotly
from flask import Flask, render_template, request, jsonify
from fuzzy_logic.terms import Term
from fuzzy_logic.variables import FuzzyVariable, SugenoVariable, LinearSugenoFunction
from fuzzy_logic.sugeno_fs import SugenoFuzzySystem
from fuzzy_logic.mf import TriangularMF
import plotly.graph_objs as go
from plotly.subplots import make_subplots

app = Flask(__name__)

zadana = 50
tsim = 1000
Cd = 1.17
m = 0.0027
pa = 1.225
A = 0.0012
g = 9.8
h_min = 0.0
h_max = 100.0
Radius = 0.15

RPM_pid = [0.0]
RPM_fuzzy = [0.0]
RMP_max = 1800.0
RMP_min = 0.0

Tp_pid = 0.1
h_pid = [0.0]
t_pid = [0.0]
v_pid = [0.0]
vr_pid = [0.0]
Fg_pid = [m * g]
Fd_pid = [0.0]
N_pid = int(tsim / Tp_pid) + 1

v_min_pid = 0.0
v_max_pid = 3.0
u_min_pid = 0.0
u_max_pid = 5.0
kp_pid = 0.1
Ti_pid = 10
Td_pid = 2
u_PID_pid = [0.0]
e_n_pid = [zadana - h_pid[-1]]

Tp_fuzzy = 0.1
h_fuzzy = [0.0]
t_fuzzy = [0.0]
v_fuzzy = [0.0]
Fg_fuzzy = [m * g]
Fd_fuzzy = [0.0]
vx_fuzzy = [0.0]
asd = [0.0]

t1_fuzzy = Term('NB', TriangularMF(-h_max, -h_max, -h_max / 10))
t2_fuzzy = Term('NS', TriangularMF(-h_max, -h_max / 10, 0))
t3_fuzzy = Term('Z', TriangularMF(-h_max / 10, 0, h_max / 10))
t4_fuzzy = Term('PS', TriangularMF(0, h_max / 10, h_max))
t5_fuzzy = Term('PB', TriangularMF(h_max / 10, h_max, h_max))

e_fuzzy = FuzzyVariable('e', -h_max, h_max, t1_fuzzy, t2_fuzzy, t3_fuzzy, t4_fuzzy, t5_fuzzy)
u_fuzzy = SugenoVariable(
    'u',
    LinearSugenoFunction('NB', {e_fuzzy: -2}, -2),
    LinearSugenoFunction('NS', {e_fuzzy: -0.5}, -0.5),
    LinearSugenoFunction('Z', {e_fuzzy: 0}, 0),
    LinearSugenoFunction('PS', {e_fuzzy: 0.5}, 0.5),
    LinearSugenoFunction('PB', {e_fuzzy: 2}, 2)
)

FS = SugenoFuzzySystem([e_fuzzy], [u_fuzzy])
FS.rules.extend([
    FS.parse_rule('if (e is NB) then (u is NB)'),
    FS.parse_rule('if (e is NS) then (u is NS)'),
    FS.parse_rule('if (e is Z) then (u is Z)'),
    FS.parse_rule('if (e is PS) then (u is PS)'),
    FS.parse_rule('if (e is PB) then (u is PB)')
])

e_n_fuzzy = [h_max - h_fuzzy[-1]]
result_fuzzy = FS.calculate({e_fuzzy: e_n_fuzzy[-1]})
u_n_fuzzy = [result_fuzzy[u_fuzzy]]

N_fuzzy = int(tsim / Tp_fuzzy) + 1


def update_values():
    global zadana, kp_pid, Ti_pid, Td_pid, h_pid, t_pid, RPM_pid, h_fuzzy, t_fuzzy, RPM_fuzzy

    zadana = float(request.form['zadana'])
    kp_pid = float(request.form['kp'])
    Ti_pid = float(request.form['ti'])
    Td_pid = float(request.form['td'])

    h_pid = [0.0]
    t_pid = [0.0]
    RPM_pid = [0.0]

    h_fuzzy = [0.0]
    t_fuzzy = [0.0]
    RPM_fuzzy = [0.0]

    for n in range(1, N_pid):
        t_pid.append(n * Tp_pid)
        Fd_pid.append(0.5 * pa * v_pid[-1] ** 2 * Cd * A)
        Fg_pid.append(m * g)
        v_pid.append(((v_max_pid - v_min_pid) / (u_max_pid - u_min_pid)) * (u_PID_pid[-1] - u_min_pid) + v_min_pid)
        h_pid.append(min(max(Tp_pid * (Fd_pid[-1] - Fg_pid[-1]) / m + h_pid[-1], h_min), h_max))
        e_n_pid.append(zadana - h_pid[-1])
        u_PID_pid.append(kp_pid * (e_n_pid[-1] + (Tp_pid / Ti_pid * sum(e_n_pid) + (Td_pid * (e_n_pid[-1] - e_n_pid[-2]) / Tp_pid))))
        RPM_pid.append(min(max(v_pid[-1] * 60 / (2 * 3.14 * Radius), RMP_min), RMP_max))

    for n in range(1, N_fuzzy):
        t_fuzzy.append(n * Tp_fuzzy)
        Fd_fuzzy.append(0.5 * pa * v_fuzzy[-1] ** 2 * Cd * A)
        Fg_fuzzy.append(m * g)
        v_fuzzy.append(u_n_fuzzy[-1])
        h_fuzzy.append(min(max(Tp_fuzzy * (v_fuzzy[-1] - Fg_fuzzy[-1]) / g + h_fuzzy[-1], h_min), h_max))
        e_n_fuzzy.append(zadana - h_fuzzy[-1])
        result_fuzzy = FS.calculate({e_fuzzy: e_n_fuzzy[-1]})
        u_n_fuzzy.append(result_fuzzy[u_fuzzy])

        target_value = list(map(int, h_fuzzy))[n]
        index_in_pid = next((i for i, value in enumerate(map(int, h_pid)) if abs(value - target_value) <= 1), None)
        if index_in_pid is not None:
            RPM_fuzzy.append(RPM_pid[index_in_pid])
        else:
            RPM_fuzzy.append(0.0)


@app.route('/')
def index():
    return render_template('index.html')


@app.route('/update_simulation', methods=['POST'])
def update_simulation():
    update_values()

    fig = make_subplots(rows=3, cols=1, subplot_titles=['Symulacja Regulacji PID', 'Symulacja Regulacji Fuzzy PID', 'Zmiana RPM w czasie'])
    fig.add_trace(go.Scatter(x=t_pid, y=h_pid, mode='lines', name='Wysokość PID'), row=1, col=1)
    fig.add_trace(go.Scatter(x=t_pid, y=[zadana] * len(t_pid), mode='lines', line=dict(dash='dash'), name='Wartość zadana PID'), row=1, col=1)
    fig.add_trace(go.Scatter(x=t_fuzzy, y=h_fuzzy, mode='lines', name='Wysokość Fuzzy PID'), row=2, col=1)
    fig.add_trace(go.Scatter(x=t_fuzzy, y=[zadana] * len(t_fuzzy), mode='lines', line=dict(dash='dash'), name='Wartość zadana Fuzzy PID'), row=2, col=1)
    fig.add_trace(go.Scatter(x=t_pid, y=RPM_pid, mode='lines', name='RPM PID'), row=3, col=1)
    fig.add_trace(go.Scatter(x=t_fuzzy, y=RPM_fuzzy, mode='lines', name='RPM Fuzzy PID'), row=3, col=1)

    graphJSON = json.dumps(fig, cls=plotly.utils.PlotlyJSONEncoder)
    return graphJSON


if __name__ == '__main__':
    app.run(debug=True)
