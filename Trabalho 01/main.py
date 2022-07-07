
from src.problems import MazeProblem
from src.viewer import MazeViewer
from src.search import *
import time


def main():
    maze_problem = MazeProblem(300, 300)
    viewer = MazeViewer(maze_problem, step_time_miliseconds=20, zoom=20)

    print(
        f"== Trabalho 01 - Algoritmos de Buscas ==\n"
        f" 1 - Deep-first Search\n"
        f" 2 - Uniform-cost Search\n"
        f" 3 - A*\n"
        f" 4 - Extra - Breadth-first Search\n"
    )
    escolha = int(input("Escolhe: "))
    if escolha == 1:
        tempo_inicial = time.process_time()
        caminho, custo_total, nos_expandidos = deep_first_search(maze_problem, viewer)
        tempo_final = time.process_time()
            
    elif escolha == 2:
        tempo_inicial = time.process_time()
        caminho, custo_total, nos_expandidos = uniform_cost_search(maze_problem, viewer)
        tempo_final = time.process_time()
    
    elif escolha == 3:
        tempo_inicial = time.process_time()
        caminho, custo_total, nos_expandidos = astar_search(maze_problem, viewer)
        tempo_final = time.process_time()

    elif escolha == 4:
        tempo_inicial = time.process_time()
        caminho, custo_total, nos_expandidos = breadth_first_search(maze_problem, viewer)
        tempo_final = time.process_time()

    if len(caminho) == 0:
        print("\nGoal is unreachable for this maze.\n")

    print(
        f"\nCusto total do caminho: {custo_total:.2f}.\n"
        f"Numero de passos: {len(caminho)-1}.\n"
        f"Numero de nos expandidos: {nos_expandidos}.\n"
        f"Tempo de Execução (s): {tempo_final - tempo_inicial:.2f}."
    )

    #viewer.update(path=caminho)
    #viewer.pause()

    print("OK!")


if __name__ == "__main__":
    main()
