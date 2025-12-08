📋 RELATÓRIO DE ANÁLISE: Erro no Launch do Gazebo com Controladores

  1. RESUMO EXECUTIVO

  O arquivo gazebo_controllers_launch.launch.py funciona e consegue iniciar o Gazebo com o manipulador e os controladores, porém apresenta um erro de plugin devido a uma condição de
   corrida (race condition) e conflito de gerenciamento do controller_manager.

  Causa Raiz: O sistema tenta carregar controladores de duas formas diferentes simultaneamente:
  1. Via plugin do Gazebo (embutido no URDF)
  2. Via nó ros2_control_node lançado pelo launch file

  Quando você executa os controladores em um terminal separado, apenas uma dessas abordagens é usada, eliminando o conflito.

  ---
  2. ANÁLISE DETALHADA DO PROBLEMA

  2.1 Arquitetura Conflitante

  O launch file atual possui uma arquitetura híbrida que causa o erro:

  ┌─────────────────────────────────────────────────────────────────┐
  │                    CONFLITO DE CONTROLLER_MANAGER               │
  ├─────────────────────────────────────────────────────────────────┤
  │                                                                 │
  │  CAMINHO A (Via URDF/Gazebo Plugin):                           │
  │  ┌─────────────────────────────────────────────────────┐       │
  │  │ manipulator_description_gazebo.urdf (linha 508-514) │       │
  │  │ <gazebo>                                             │       │
  │  │   <plugin filename="libgz_ros2_control-system.so"   │       │
  │  │          name="gz_ros2_control::...">               │       │
  │  │     <parameters>/ros2_ws/src/.../controllers.yaml   │  ❌   │
  │  │   </plugin>                                         │       │
  │  │ </gazebo>                                           │       │
  │  └─────────────────────────────────────────────────────┘       │
  │           ↓ Tenta criar controller_manager                     │
  │                                                                 │
  │  CAMINHO B (Via Launch File):                                  │
  │  ┌─────────────────────────────────────────────────────┐       │
  │  │ gazebo_controllers_launch.launch.py (linha 48-58)   │       │
  │  │ controller_manager = Node(                          │       │
  │  │     package='controller_manager',                   │       │
  │  │     executable='ros2_control_node',                 │  ❌   │
  │  │     name='controller_manager',                      │       │
  │  │     ...                                             │       │
  │  │ )                                                   │       │
  │  └─────────────────────────────────────────────────────┘       │
  │           ↓ Também tenta criar controller_manager              │
  │                                                                 │
  │  RESULTADO: DOIS controller_managers competindo!               │
  └─────────────────────────────────────────────────────────────────┘

  2.2 Problemas Identificados no Código

  Problema 1: Path Hardcoded no URDF (linha 511)

  <!-- manipulator_description_gazebo.urdf:511 -->
  <parameters>/ros2_ws/src/manipulator_controllers_pkg/controllers/controllers.yaml</parameters>

  Este path /ros2_ws/ só existe dentro do container Docker. No host, este arquivo não é encontrado, gerando erro.

  Problema 2: Dois Mecanismos de Carregamento Paralelos

  O launch file define dois conjuntos de carregamento de controladores:

  Conjunto 1 - TimerAction (linhas 81-89):
  # Delay para garantir que o Gazebo carregou o plugin
  delayed_load_joint_state = TimerAction(period=5.0, ...)
  delayed_load_velocity = TimerAction(period=7.0, ...)
  Estes são adicionados incondicionalmente ao LaunchDescription (linhas 141-142).

  Conjunto 2 - Event Handlers (linhas 92-116):
  spawn_entity_handler = RegisterEventHandler(
      condition=IfCondition(LaunchConfiguration('start_controller_manager'))
  )
  controller_manager_start_handler = RegisterEventHandler(
      condition=IfCondition(LaunchConfiguration('start_controller_manager'))
  )
  Estes são condicionais baseados em start_controller_manager.

  O problema: Os TimerAction (linhas 141-142) são executados SEMPRE, independentemente do valor de start_controller_manager:

  return LaunchDescription([
      # ...
      delayed_load_joint_state,  # SEMPRE EXECUTA (sem condição!)
      delayed_load_velocity,     # SEMPRE EXECUTA (sem condição!)
  ])

  Problema 3: Condição de Corrida no Sequenciamento

  TIMELINE DO PROBLEMA:
  ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  t=0s    │ Gazebo inicia
  t=1-2s  │ Plugin tenta ler /ros2_ws/... (FALHA - path não existe)
  t=?s    │ Entity spawned → controller_manager (ros2_control_node) inicia
  t=?s    │ OnProcessStart → load_joint_state_controller
  t=5s    │ TimerAction → load_joint_state_controller (DUPLICADO!)
  t=?s    │ OnProcessExit → load_velocity_controller
  t=7s    │ TimerAction → load_velocity_controller (DUPLICADO!)
  ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

  ---
  3. POR QUE FUNCIONA EM TERMINAL SEPARADO?

  Quando você executa os controladores em terminal separado:

  1. Você provavelmente usa start_controller_manager:=false no launch do Gazebo
  2. Isso faz o Gazebo tentar usar seu próprio controller_manager (via plugin)
  3. O plugin falha ao ler o path hardcoded
  4. Você então inicia o ros2_control_node manualmente
  5. Carrega os controladores manualmente com ros2 control load_controller

  Resultado: Apenas um controller_manager está ativo, sem conflitos.

  ---
  4. ERROS ESPERADOS

  Os erros que você provavelmente vê incluem:

  [ERROR] [gz_ros2_control]: Failed to load parameter file: /ros2_ws/src/.../controllers.yaml
  [WARN] [controller_manager]: Controller 'joint_state_broadcaster' already loaded
  [ERROR] [spawner]: Controller 'velocity_controller' failed to configure

  ---
  5. DIAGRAMA DA ARQUITETURA ATUAL vs CORRETA

  Arquitetura Atual (COM PROBLEMA):

  ┌────────────────────┐     ┌────────────────────┐
  │   Gazebo Plugin    │     │  ros2_control_node │
  │ (controller_mgr 1) │     │ (controller_mgr 2) │
  └─────────┬──────────┘     └─────────┬──────────┘
            │                          │
            └──────────┬───────────────┘
                       │
                ┌──────▼──────┐
                │   CONFLITO  │
                │  2 managers │
                └─────────────┘

  Arquitetura Correta (ESCOLHER UMA):

  OPÇÃO A: Plugin do Gazebo          OPÇÃO B: ros2_control_node
  ┌────────────────────────┐         ┌────────────────────────┐
  │      Gazebo Plugin     │         │    ros2_control_node   │
  │   (controller_manager) │         │   (controller_manager) │
  │                        │   OU    │                        │
  │ Corrigir path no URDF  │         │ Remover plugin do URDF │
  └────────────────────────┘         └────────────────────────┘

  ---
  6. RECOMENDAÇÕES DE CORREÇÃO

  Opção A: Usar Apenas ros2_control_node (RECOMENDADO)

  1. Remover o bloco <gazebo> do URDF (linhas 508-514)
  2. Remover os TimerActions incondicionais do launch file
  3. Manter apenas os Event Handlers condicionais

  Opção B: Usar Apenas Plugin do Gazebo

  1. Corrigir o path no URDF para usar path dinâmico:
  <parameters>$(find manipulator_controllers_pkg)/controllers/controllers.yaml</parameters>
  1. Nota: Gazebo Sim não suporta $(find ...) diretamente - requer workaround.
  2. Remover o nó controller_manager do launch file
  3. Adicionar condição aos TimerActions

  Opção C: Corrigir o Launch File Atual (Mínima Alteração)

  Adicionar condição aos TimerActions para só executarem quando start_controller_manager=false:

  # Mudança necessária nas linhas 141-142:
  ConditionalTimerAction(
      condition=UnlessCondition(LaunchConfiguration('start_controller_manager')),
      period=5.0,
      actions=[load_joint_state_controller],
  )

  ---
  7. TABELA RESUMO

  | Aspecto               | Estado Atual           | Impacto                |
  |-----------------------|------------------------|------------------------|
  | Path do YAML no URDF  | Hardcoded /ros2_ws/... | ❌ Falha fora do Docker |
  | TimerActions          | Sem condição           | ❌ Executa duplicado    |
  | Event Handlers        | Com condição           | ✅ Correto              |
  | Dois mecanismos de CM | Ambos ativos           | ❌ Conflito             |

  ---
  8. CONCLUSÃO

  O launch file funciona porque o ros2_control_node consegue inicializar o sistema mesmo com o erro do plugin. Porém, o erro do plugin é causado por:

  1. Path hardcoded específico para Docker
  2. Execução duplicada de comandos de carregamento de controladores
  3. Conflito entre o plugin do Gazebo e o nó ros2_control_node

  A solução mais limpa é escolher uma única abordagem (recomendo ros2_control_node) e remover a outra completamente do sistema.
