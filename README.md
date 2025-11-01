NAVIQ – Muito mais que um robô
Introdução

O NAVIQ é um protótipo de robô móvel de pequeno porte, construído de forma robusta e escalável, utilizando tecnologias modernas como ROS 2 Jazzy e Gazebo. Seu principal objetivo é possibilitar o controle e monitoramento de ambientes em tempo real, especialmente em locais de difícil acesso ou com riscos à integridade humana, realizando inspeções seguras em terrenos potencialmente perigosos.

Imagine uma empresa de mineração que utiliza explosivos para romper camadas subterrâneas e criar novos caminhos. Após uma detonação, o ambiente pode conter gases tóxicos — como monóxido de carbono (CO) e metano (CH₄) — altamente inflamáveis e prejudiciais à saúde. Em 19 de setembro de 2025, um trabalhador no Sul da Bahia faleceu em decorrência de uma explosão súbita em uma mina. Casos como esse podem ser evitados com o uso do NAVIQ.
Equipado com sensores de gases e uma câmera capaz de capturar imagens a 20 quadros por segundo em resolução 720p, o robô é capaz de identificar riscos de explosão e monitorar a qualidade do ar, avaliando se o ambiente é seguro para a entrada de pessoas.

Conceitos técnicos – Interface Gráfica

O sistema do NAVIQ é composto por uma interface gráfica (GUI) desenvolvida em Python, responsável por interagir diretamente com o usuário. Essa interface possui:

Um espaço dedicado à transmissão da câmera do robô;

Um analógico virtual (knob) para controle de direção e velocidade;

Três labels informativas no canto superior direito: duas exibindo as leituras dos sensores de gás em partes por milhão (PPM) e uma indicando o modo de navegação (Forte, Normal ou Turbo).

Os arquivos da interface estão organizados na pasta gui.
O sistema permite a conexão de joysticks compatíveis com o computador, como o Gamesir T4, utilizado neste projeto. Ao pressionar o botão de aceleração, a interface envia as coordenadas cartesianas 2D do analógico, determinando a direção e intensidade do movimento desejado.

Com base na posição do analógico, são calculadas as velocidades linear e angular, publicadas no tópico cmd_vel. Dentro da mesma rede, o Raspberry Pi, atuando como o robô, recebe essas mensagens quase em tempo real via DDS (Data Distribution Service), garantindo uma comunicação rápida e confiável entre a GUI e o sistema embarcado.

Abaixo, a arquitetura de alto nível da interface:

<img width="1015" height="511" alt="image" src="https://github.com/user-attachments/assets/5ce7edd1-b529-420c-9168-14c08a835f99" />
Conceitos técnicos – ROS 2 e Raspberry Pi

Após o envio dos comandos pela interface, o ROS 2 é responsável por traduzi-los em ações físicas. O ROS 2 (Robot Operating System 2) é um framework de código aberto voltado para o desenvolvimento de sistemas robóticos. Ele fornece bibliotecas, ferramentas e padrões que facilitam a comunicação entre sensores, atuadores e módulos de controle, permitindo que o sistema seja modular, escalável e reutilizável.

O projeto utiliza a versão Jazzy, mais recente do ROS 2, que oferece melhorias na performance, segurança e compatibilidade com simuladores como o Gazebo.

No diretório src, encontra-se o pacote principal do sistema: minebot_playground, onde estão todos os nós, arquivos de configuração, modelos do robô e scripts de simulação.

Simulador

O simulador foi construído no Gazebo, um ambiente virtual 3D utilizado para testar o robô e validar o comportamento dos nós ROS 2 antes da execução em hardware real.

O arquivo naviq.urdf, localizado na pasta urdf, descreve as características físicas e geométricas do robô, como dimensões, sensores e juntas.
Já o arquivo world_mine.sdf, presente na pasta worlds, define o mundo virtual da simulação, que representa uma mina subterrânea com obstáculos e paredes, simulando o cenário de operação real do NAVIQ.

<img width="367" height="318" alt="image" src="https://github.com/user-attachments/assets/502747aa-58a6-4ba7-b76a-d0dc16bd03c0" /> <img width="568" height="834" alt="image" src="https://github.com/user-attachments/assets/bbc5080a-3208-4bc9-8680-e236742a4b3a" />

O arquivo launch.py é responsável por iniciar a simulação no Gazebo. Ele carrega o modelo URDF do robô, inicializa o simulador, publica o estado do robô por meio do robot_state_publisher, insere o modelo no ambiente e cria uma ponte de comunicação entre ROS 2 e Gazebo — garantindo a troca de mensagens, como comandos de velocidade (cmd_vel) e dados de odometria (/odom).

Nós ROS 2 e Integração com o Hardware

No ROS 2, um nó é uma unidade independente de execução que realiza uma função específica dentro do sistema.
Cada nó pode publicar (enviar) ou assinar (receber) mensagens por meio de tópicos, o que permite que diferentes componentes do robô se comuniquem de forma modular.

No NAVIQ, três nós principais foram desenvolvidos e executam papéis complementares dentro do Raspberry Pi, que é o cérebro do robô: processa dados dos sensores, controla os motores e envia informações à interface gráfica.

`Nó gas_mock`

O nó gas_mock simula os sensores de gases que futuramente serão instalados no robô, como o sensor de monóxido de carbono (CO) e o sensor de metano (CH₄).

Ele gera valores aleatórios representando as leituras dos sensores e os publica periodicamente em dois tópicos:

gas/co2_ppm — concentrações de CO;

gas/ch4_ppm — concentrações de CH₄.

Esses dados são publicados como mensagens Float32, e a interface gráfica atua como subscriber, recebendo as informações e exibindo-as em tempo real.
Atualmente, o gas_mock permite simular o comportamento dos sensores antes da integração física com o hardware.

`Nó user_controller`

O nó user_controller é responsável por traduzir os comandos de movimento enviados pela interface gráfica em sinais elétricos que acionam os motores do robô.

Ele é um subscriber do tópico cmd_vel, recebendo mensagens Twist com os valores de velocidade linear e angular.
Esses valores são processados no método listener_callback, que calcula a velocidade individual de cada roda (esquerda e direita), compensando giros e curvas.

O controle físico dos motores é realizado com a biblioteca gpiozero, que fornece acesso direto aos pinos GPIO do Raspberry Pi.
No código, os motores são definidos da seguinte forma:

left_motor = Motor(forward=17, backward=27, enable=12)
right_motor = Motor(forward=22, backward=23, enable=13)


Esses pinos controlam os drivers dos motores, que convertem os sinais digitais em corrente elétrica para girar as rodas do robô.
Assim, o Raspberry Pi atua como controlador central, recebendo comandos da interface (via ROS 2) e movimentando fisicamente o robô em resposta.

`Nó usbcam_node`

O nó usbcam_node é responsável por capturar imagens em tempo real da câmera acoplada ao robô (USB ou câmera nativa do Raspberry Pi).

Ele utiliza a biblioteca OpenCV para capturar os frames e o CvBridge para convertê-los em mensagens ROS do tipo CompressedImage.
Essas imagens são publicadas continuamente no tópico usbcam_node/compressed em formato JPEG, a uma taxa de aproximadamente 20 FPS.
A interface gráfica, como subscriber, recebe esses frames e os exibe em tempo real para o operador.

Integração dos Nós

O sistema completo utiliza o modelo de comunicação publisher/subscriber, no qual cada nó tem uma função específica e interage de forma assíncrona.

Nó	Função	Tipo	Tópico(s)	Relação com o Hardware	Relação com a Interface
gas_mock	Simulação de sensores de gás	Publisher	gas/co2_ppm, gas/ch4_ppm	Futuro uso com sensores reais ligados ao Raspberry Pi	Interface exibe os valores
user_controller	Controle dos motores	Subscriber	cmd_vel	Aciona motores via GPIO do Raspberry Pi	Recebe comandos de direção
usbcam_node	Captura e envio de vídeo	Publisher	usbcam_node/compressed	Usa câmera conectada ao Raspberry Pi	Exibe vídeo ao usuário
Conclusão

O desenvolvimento do NAVIQ demonstrou a capacidade do ROS 2 como plataforma de integração entre hardware e software em sistemas robóticos distribuídos.
A arquitetura projetada possibilita o controle remoto, a visualização em tempo real e o monitoramento ambiental com base em sensores simulados e câmeras embarcadas.

O uso do Raspberry Pi como unidade central de processamento permitiu a execução simultânea de múltiplos nós ROS 2, garantindo a comunicação entre os módulos de controle, percepção e visualização.
Mesmo com sensores simulados, o ecossistema mostrou-se estável, escalável e pronto para receber componentes reais, como sensores de gases, novos atuadores e câmeras de maior resolução.

Com a substituição dos dados mockados por leituras reais e a calibração dos motores, o NAVIQ estará apto para atuar em ambientes de risco — como minas subterrâneas —, reduzindo a exposição humana e aumentando a segurança operacional.
Assim, o projeto cumpre seu objetivo principal: provar que um robô acessível, modular e inteligente pode salvar vidas e transformar o modo como exploramos ambientes hostis.
