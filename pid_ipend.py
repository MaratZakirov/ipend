import time
import gymnasium as gym
import numpy as np

def evaluate_pid(params, max_steps=1000):
    """
    Запускает один тренировочный эпизод БЕЗ графики.
    Возвращает суммарную ошибку (чем меньше ошибка, тем лучше PID).
    """
    Kp, Ki, Kd = params
    # Создаем быструю среду в памяти без отрисовки окна
    env = gym.make("InvertedPendulum-v5", render_mode=None)
    observation, info = env.reset()

    total_error = 0.0
    integral = 0.0
    prev_error = 0.0
    dt = 0.02  # Стандартный шаг времени для MuJoCo InvertedPendulum

    for _ in range(max_steps):
        # В MuJoCo InvertedPendulum-v5:
        # observation[0] - позиция тележки
        # observation[1] - угол наклона маятника (в радианах)
        # Наша цель: удерживать угол theta = 0.0
        theta = observation[1]
        error = 0.0 - theta

        integral += error * dt
        derivative = (error - prev_error) / dt
        prev_error = error

        # Вычисляем непрерывную силу PID
        u = Kp * error + Ki * integral + Kd * derivative

        # MuJoCo требует action в виде numpy-массива формы (1,)
        action = np.array([u], dtype=np.float32)

        # Делаем шаг в среде
        observation, reward, terminated, truncated, info = env.step(action)

        # Штрафуем за отклонение угла (квадрат ошибки)
        total_error += error ** 2

        if terminated or truncated:
            # Если маятник упал раньше времени, добавляем огромный штраф за оставшиеся шаги
            total_error += (max_steps - _) * 100.0
            break

    env.close()
    return total_error

def train_pid_twiddle(tol=0.01):
    """Алгоритм Twiddle (умный перебор по сетке) для поиска Kp, Ki, Kd"""
    print("Начинаем подбор параметров PID через Twiddle...")

    # Стартовые коэффициенты [Kp, Ki, Kd]
    p = [1.0, 0.0, 1.0]
    # Стартовый шаг изменения для каждого коэффициента [dKp, dKi, dKd]
    dp = [1.0, 0.1, 1.0]

    best_err = evaluate_pid(p)

    # Крутим цикл, пока шаги подбора не станут совсем крошечными (меньше tol)
    while sum(dp) > tol:
        for i in range(len(p)):
            p[i] += dp[i]  # Пробуем увеличить коэффициент
            err = evaluate_pid(p)

            if err < best_err:
                best_err = err
                dp[i] *= 1.1  # Успех! Увеличиваем шаг поиска
            else:
                p[i] -= 2 * dp[i]  # Не вышло, пробуем уменьшить
                err = evaluate_pid(p)

                if err < best_err:
                    best_err = err
                    dp[i] *= 1.1  # Успех в другую сторону!
                else:
                    p[i] += dp[i]  # Возвращаем коэффициент на место
                    dp[i] *= 0.9  # Сжимаем область поиска для этого параметра

    print(f"Подбор завершен! Лучшие коэффициенты: Kp={p[0]:.2f}, Ki={p[1]:.2f}, Kd={p[2]:.2f}")
    return p


def run_demonstration(best_params):
    """Запуск красивой демонстрации обученного PID в режиме 'human'"""
    print("\nЗапуск демонстрации в окне MuJoCo...")
    Kp, Ki, Kd = best_params

    env = gym.make("InvertedPendulum-v5", render_mode="human")
    observation, info = env.reset()

    integral = 0.0
    prev_error = 0.0
    dt = 0.02

    for _ in range(1000):
        # Берем текущий угол маятника
        theta = observation[1]
        error = 0.0 - theta

        integral += error * dt
        derivative = (error - prev_error) / dt
        prev_error = error

        # Считаем силу управления
        u = Kp * error + Ki * integral + Kd * derivative

        # Передаем массив в среду
        action = np.array([u], dtype=np.float32)
        observation, reward, terminated, truncated, info = env.step(action)

        time.sleep(0.02)

        if terminated or truncated:
            print("Маятник упал или время вышло! Перезапуск...")
            observation, info = env.reset()
            integral = 0.0
            prev_error = 0.0

    env.close()

if __name__ == "__main__":
    # 1. Запускаем "слепой", но быстрый перебор параметров на основе физики среды
    best_pid_coefficients = train_pid_twiddle()

    # 2. Смотрим на результат его "мозгов" в реальном времени
    run_demonstration(best_pid_coefficients )