
<h1>CSharpDroneLib</h1>
<p>O CSharpDroneLib é uma biblioteca Open Source para criação de funções de controle de drones, seguindo o protocolo MavLink. Nosso objetivo é permitir que pessoas ao redor do mundo possam desenvolver softwares de maneira mais eficiente, seguindo os protocolos adequados.
</p>
<p>Para utilizar esta biblioteca em seu projeto, basta adicionar a DLL como referência e utilizar o exemplo contido nos arquivos como base para a construção do seu software. Para testes, utilizamos o SITL (Software In the Loop), e o projeto em si foi desenvolvido com base em uma engenharia reversa do DroneKit.
</p>
<ul>
  <li>TakeOff</li>
  <li>Land</li>
  <li>GoToWaypoint</li>
  <li>SetFlightMode</li>
  <li>Arm</li>
  <li>Disarm</li>
  <li>Connect</li>
  <li>Disconnect</li>
</ul>
Atualmente, a biblioteca oferece acesso aos seguintes dados do drone:

<ul>
  <li>Altitude</li>
  <li>Latitude</li>
  <li>Longitude</li>
  <li>Rollspeed</li>
  <li>Yawspeed</li>
  <li>Pitchspeed</li>
  <li>Roll</li>
  <li>Pitch</li>
  <li>Yaw</li>
  <li>BatteryLevel</li>
  <li>FlightMode</li>
</ul>

<ul>
  Atualmente contamos com os seguintes modos de voo:
  <li>Stabilized   </li>
  <li>AltitudeHold </li>
  <li>Loiter       </li>
  <li>Guided       </li>
  <li>Auto         </li>
  <li>Land         </li>
  <li>ReturnToHome </li>
</ul>

<ul>
  Atualmente contamos com os seguintes modos de conexao:
  <li>UDP    </li>
  <li>TCP    </li>
  <li>Serial </li>
</ul>
