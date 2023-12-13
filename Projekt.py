import streamlit as st
import matplotlib.pyplot as plt
from fuzzy_logic.terms import Term
from fuzzy_logic.variables import FuzzyVariable, SugenoVariable, LinearSugenoFunction
from fuzzy_logic.sugeno_fs import SugenoFuzzySystem
from fuzzy_logic.mf import TriangularMF
import math


def plot(h_given, kp_pid, Ti_pid, Td_pid):
    tsim = 1000  # Czas trwania symulacji [s]
    Cd = 1.17  # Współczynnik oporu powietrza
    m = 0.0027  # Masa standardowej piłki do tenisa stołowego [kg]
    pa = 1.225  # Gęstość powietrza [kg/m^3]
    A = 0.0012  # Powierzchnia przekroju standardowej piłki do tenisa stołowego [m^2]
    g = 9.8  # Przyspieszenie ziemskie [m/s^2]
    h_min = 0.0  # Minimalna wysokość [mm]
    h_max = 100.0  # Maksymalna wysokość [mm]
    Radius = 0.15  # Promień wiatraka [m]

    RPM_pid = [0.0]  # RPM dla regulatora PID
    RPM_fuzzy = [0.0]  # RPM dla regulatora Fuzzy PID
    RMP_max = 1800.0  # Maksymalne RMP wytwarzane przez domowy wiatrak o promieniu 15[cm]
    RMP_min = 0.0  # Mininalna wartość RMP

    # Parametry procesu PID

    Tp_pid = 0.1  # Krok czasowy symulacji regulatora PID [s]
    h_pid = [0.0]  # Wysokość piłki [mm]
    t_pid = [0.0]  # Czas [s]
    v_pid = [0.0]  # Prędkość piłki [m/s]
    Fg_pid = [m * g]  # Siła grawitacyjna działająca na piłkę [N]
    Fd_pid = [0.0]  # Siła oporu powietrza [N]
    N_pid = int(tsim / Tp_pid) + 1  # Ilość kroków czasowych

    # Ustawienia regulatora PID

    v_min_pid = 0.0  # Minimalna prędkość dla regulatora PID [m/s]
    v_max_pid = 3.0  # Maksymalna prędkość dla regulatora PID [m/s]
    u_min_pid = 0.0  # Minimalna wartość sterowania dla regulatora PID
    u_max_pid = 5.0  # Maksymalna wartość sterowania dla regulatora PID
    u_PID_pid = [0.0]  # Wartość sterowania regulatora PID
    e_n_pid = [h_given - h_pid[-1]]  # Uchyb regulacji regulatora PID

    # Parametry procesu Fuzzy PID

    Tp_fuzzy = 0.1  # Krok czasowy symulacji regulatora Fuzzy PID [s]
    h_fuzzy = [0.0]  # Wysokość piłki [m]
    t_fuzzy = [0.0]  # Czas [s]
    v_fuzzy = [0.0]  # Prędkość piłki [m/s]
    Fg_fuzzy = [m * g]  # Siła grawitacyjna działająca na piłkę [N]
    Fd_fuzzy = [0.0]  # Siła oporu powietrza [N]

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
    FS.rules.append(FS.parse_rule('if (e is NB) then (u is NB)'))
    FS.rules.append(FS.parse_rule('if (e is NS) then (u is NS)'))
    FS.rules.append(FS.parse_rule('if (e is Z) then (u is Z)'))
    FS.rules.append(FS.parse_rule('if (e is PS) then (u is PS)'))
    FS.rules.append(FS.parse_rule('if (e is PB) then (u is PB)'))

    e_n_fuzzy = [h_max - h_fuzzy[-1]]
    result_fuzzy = FS.calculate({e_fuzzy: e_n_fuzzy[-1]})
    u_n_fuzzy = [result_fuzzy[u_fuzzy]]
    N_fuzzy = int(tsim / Tp_fuzzy) + 1

    #Regulator PID
    for n in range(1, N_pid):
        t_pid.append(n * Tp_pid)
        Fd_pid.append(0.5 * pa * v_pid[-1] ** 2 * Cd * A)
        Fg_pid.append(m * g)

        v_pid.append(((v_max_pid - v_min_pid) / (u_max_pid - u_min_pid)) * (u_PID_pid[-1] - u_min_pid) + v_min_pid)

        h_pid.append(min(max(Tp_pid * (Fd_pid[-1] - Fg_pid[-1]) / m + h_pid[-1], h_min), h_max))

        e_n_pid.append(h_given - h_pid[-1])
        u_PID_pid.append(kp_pid * (e_n_pid[-1] + (Tp_pid / Ti_pid * sum(e_n_pid) + (Td_pid * (e_n_pid[-1] - e_n_pid[-2]) / Tp_pid))))

        RPM_pid.append(min(max(v_pid[-1] * 60 / (2 * math.pi * Radius), RMP_min), RMP_max))
    # Regulator Rozmyty
    for n in range(1, N_fuzzy):
        t_fuzzy.append(n * Tp_fuzzy)
        Fd_fuzzy.append(0.5 * pa * v_fuzzy[-1] ** 2 * Cd * A)
        Fg_fuzzy.append(m * g)

        v_fuzzy.append(u_n_fuzzy[-1])
        h_fuzzy.append(min(max(Tp_fuzzy * (v_fuzzy[-1] - Fg_fuzzy[-1]) / g + h_fuzzy[-1], h_min), h_max))

        e_n_fuzzy.append(h_given - h_fuzzy[-1])
        result_fuzzy = FS.calculate({e_fuzzy: e_n_fuzzy[-1]})
        u_n_fuzzy.append(result_fuzzy[u_fuzzy])
        target_value = list(map(int, h_fuzzy))[n]
        index_in_pid = next((i for i, value in enumerate(map(int, h_pid)) if abs(value - target_value) <= 1), None)
        RPM_fuzzy.append(RPM_pid[index_in_pid])


    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 15))

    ax1.plot(t_pid, h_pid, label='Wysokość')
    ax1.plot(t_pid, [h_given] * len(t_pid), 'r--', label='Wartość zadana')
    ax1.set_xlabel('Czas [s]')
    ax1.set_ylabel('Wysokość [mm]')
    ax1.set_title('Symulacja Regulacji PID')
    ax1.legend()
    ax1.grid(True)

    ax2.plot(t_fuzzy, h_fuzzy, label='Wysokość')
    ax2.plot(t_fuzzy, [h_given] * len(t_fuzzy), 'r--', label='Wartość zadana')
    ax2.set_xlabel('Czas [s]')
    ax2.set_ylabel('Wysokość [mm]')
    ax2.set_title('Symulacja Regulacji Rozmytej PID')
    ax2.legend()
    ax2.grid(True)

    ax3.plot(t_pid, RPM_pid, label='RPM PID')
    ax3.plot(t_fuzzy, RPM_fuzzy, label='RPM Rozmyty PID')
    ax3.set_xlabel('Czas [s]')
    ax3.set_ylabel('RPM')
    ax3.set_title('Zmiana RPM w czasie')
    ax3.legend()
    ax3.grid(True)

    plt.tight_layout()
    st.pyplot(fig)


def main():
    st.title("Symulacja Regulacji PID i Rozmytej PID")

    h_given = st.number_input("Wysokość zadana:", value=50.0)
    kp_pid = st.number_input("Część proporcjonalna (P):", value=0.1)
    Ti_pid = st.number_input("Część całkująca (I):", value=10.0)
    Td_pid = st.number_input("Część różniczkująca (D):", value=2.0)

    if st.button("Zastosuj"):
        plot(h_given, kp_pid, Ti_pid, Td_pid)


if __name__ == '__main__':
    main()
