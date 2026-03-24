# ArduPilot Domain Extraction Seed

Este mapa cobre os subsistemas centrais pedidos no modo observador e usa apenas evidência localizada no código.

## Diretórios e responsabilidades

| Diretório | Papel principal | Componentes relacionados | Evidência |
| --- | --- | --- | --- |
| `libraries/AP_GPS` | Frontend de GPS com múltiplos backends seriais, MAVLink, MSP e DroneCAN | GPS | `AP_GPS.cpp`, `AP_GPS.h` |
| `libraries/AP_InertialSensor` | Frontend de IMU, calibração accel/gyro, mascaramento de instâncias, notch filters | IMU | `AP_InertialSensor.cpp`, `AP_InertialSensor_Params.cpp` |
| `libraries/AP_Baro` | Frontend de barômetro, seleção de primário, barramento externo, checks de altitude | Barometer | `AP_Baro.cpp`, `AP_Baro.h` |
| `libraries/AP_Motors` | Implementações de motores por tipo de frame, spool logic, compensação por bateria, protocolos PWM/DShot | Motors | `AP_MotorsMulticopter.cpp`, `AP_MotorsMulticopter.h` |
| `libraries/SRV_Channel` | Roteamento de saídas, limites, trim, reversão e função por canal | Servos | `SRV_Channel.cpp`, `SRV_Channels.cpp`, `SRV_Channel.h` |
| `libraries/AP_NavEKF2` | Fusão de sensores para estado, origem, saúde do filtro e lanes por IMU | EKF2 | `AP_NavEKF2.cpp`, `AP_NavEKF2.h` |
| `libraries/AP_NavEKF3` | Fusão de sensores com source sets explícitos, afinidade de lanes e suporte ampliado a fontes auxiliares | EKF3 | `AP_NavEKF3.cpp`, `AP_NavEKF3.h` |
| `libraries/AP_BattMonitor` | Frontend de energia com múltiplos backends, múltiplas instâncias e políticas de failsafe/arming | Battery | `AP_BattMonitor.cpp`, `AP_BattMonitor.h`, `AP_BattMonitor_Params.cpp` |
| `libraries/AC_AttitudeControl` | Controle de atitude e shaping de alvos angulares/rates | AttitudeControl | `AC_AttitudeControl.cpp`, `AC_AttitudeControl.h` |
| `libraries/AP_AHRS` | Camada de referência de atitude que hospeda EKF2/EKF3 e oferece views para controladores | AHRS | `AP_AHRS.h`, `AP_AHRS.cpp` |
| `libraries/AP_Vehicle` | Agrega sensores e sistemas comuns dentro do objeto de veículo | AHRS, GPS, Barometer, IMU | `AP_Vehicle.h` |

## Prefixos reais expostos por veículo

Os módulos acima não expõem necessariamente o mesmo prefixo do nome da classe. O prefixo efetivo aparece nos `Parameters.cpp` dos veículos.

| Prefixo | Origem | Módulo alvo | Observação |
| --- | --- | --- | --- |
| `GPS` | `GOBJECT(gps, "GPS", AP_GPS)` | `libraries/AP_GPS` | Prefixo sem underscore final no `Parameters.cpp`, mas parâmetros finais aparecem como `GPS_*` |
| `INS` | `GOBJECT(ins, "INS", AP_InertialSensor)` | `libraries/AP_InertialSensor` | Gera parâmetros `INS_*` |
| `BARO` | `GOBJECT(barometer, "BARO", AP_Baro)` | `libraries/AP_Baro` | Gera parâmetros `BARO_*` |
| `MOT_` | `GOBJECTVARPTR(motors, "MOT_", ...)` | `libraries/AP_Motors` | Multicopter em Copter |
| `H_` | `GOBJECTVARPTR(motors, "H_", ...)` | `libraries/AP_Motors` | Helicóptero em Copter |
| `Q_M_` | macro `AP_MOTORS_PARAM_PREFIX` | `libraries/AP_Motors` | Variante QuadPlane em Plane |
| `ATC_` | `GOBJECTVARPTR(attitude_control, "ATC_", ...)` | `libraries/AC_AttitudeControl` | Prefixo de attitude control em Copter |
| `EK2_` | `GOBJECTN(ahrs.EKF2, NavEKF2, "EK2_", NavEKF2)` | `libraries/AP_NavEKF2` | Prefixo de EKF2 em Copter, Plane e Rover |
| `EK3_` | `GOBJECTN(ahrs.EKF3, NavEKF3, "EK3_", NavEKF3)` | `libraries/AP_NavEKF3` | Prefixo de EKF3 em Copter, Plane e Rover |
| `BATT` | `GOBJECT(battery, "BATT", AP_BattMonitor)` | `libraries/AP_BattMonitor` | Prefixo base do monitor de bateria; expande em `BATT_*`, `BATT2_*`, etc. |
| `SERVO` | `AP_SUBGROUPINFO(servo_channels, "SERVO", ...)` | `libraries/SRV_Channel` | Gera grupos `SERVO1_*` a `SERVO32_*` |

## Relações estruturais confirmadas

### 1. Veículo agrega sensores básicos

`AP_Vehicle.h` declara membros concretos para:

- `AP_GPS gps;`
- `AP_Baro barometer;`
- `AP_InertialSensor ins;`

Isso faz desses componentes dependências-base de múltiplos veículos.

### 2. AHRS hospeda EKF2 e EKF3

`AP_AHRS.h` inclui `AP_NavEKF2.h` e `AP_NavEKF3.h` e declara:

- `NavEKF2 EKF2;`
- `NavEKF3 EKF3;`

Então `EKF2` e `EKF3` são backends do sistema AHRS, não sistemas isolados do resto da pilha.

### 3. AttitudeControl depende explicitamente de AHRS e Motors

O construtor em `AC_AttitudeControl.h` recebe:

- `AP_AHRS_View &ahrs`
- `AP_Motors &motors`

E `ArduCopter/system.cpp` instancia o controlador só depois de criar `motors` e `ahrs_view`.

### 4. Motors depende de servo outputs e bateria

`AP_MotorsMulticopter.cpp` inclui:

- `AP_BattMonitor/AP_BattMonitor.h`
- `SRV_Channel/SRV_Channel.h`

e expõe parâmetros `MOT_BAT_*` e `MOT_PWM_*`, mostrando dependência estrutural de monitor de bateria e roteamento de saída.

### 5. INS recebe influência operacional de motores

`AP_Vehicle.cpp` usa `AP::motors()` para atualizar harmonic notch tracking de `AP_InertialSensor::HarmonicNotch`.

Isso não torna IMU “filha” de motores, mas cria dependência operacional de configuração: notch dinâmico pode depender do throttle/motor thrust.

### 6. EKF2 consome múltiplas famílias de sensores

Em `AP_NavEKF2.cpp`, os grupos de parâmetros evidenciam famílias de entrada:

- `GPS_*`
- `ALT_*` e `HGT_*`
- `MAG_*`
- `EAS_*`
- `RNG_*`
- `FLOW_*`
- `BCN_*`
- `IMU_MASK`

Isso mostra que EKF2 funde IMU, GPS, barômetro e outros sensores auxiliares quando presentes.

### 7. EKF3 amplia a configuração explícita de fontes

Em `AP_NavEKF3.cpp`, além das famílias equivalentes às do EKF2, aparecem parâmetros e grupos adicionais como:

- `EK3_SRCn_*`
- `EK3_AFFINITY`
- `EK3_ERR_THRESH`
- `EK3_VIS_*`
- `EK3_WENC_*`

Isso mostra que o EKF3 mantém a fusão principal de IMU/GPS/barômetro/bússola, mas explicita source sets e inclui suporte parametrizado a visual odometry e wheel encoder.

### 8. Battery é um frontend multi-instância com backends variados

`AP_BattMonitor.cpp` declara grupos por instância:

- `BATT_*`
- `BATT2_*`
- `BATT3_*`
- `...`

e combina dois blocos por instância:

- `AP_BattMonitor_Params`
- backend específico selecionado por `BATT_MONITOR`

Os backends listados no próprio frontend incluem `Analog`, `SMBus`, `DroneCAN`, `ESC`, `Sum`, `FuelFlow`, `Generator`, `INA2xx`, `INA239`, `INA3221`, `LTC2946`, `Torqeedo`, `Scripting` e outros.

### 9. Battery participa de arming checks e da compensação de motores

Há evidência direta em:

- `AP_Arming.cpp`: `AP::battery().arming_checks(...)`
- `AP_MotorsMulticopter.cpp`: leitura de corrente, resistência e tensão via `AP::battery()`
- `AP_Motors_Thrust_Linearization.cpp`: uso de `voltage()` e `voltage_resting_estimate()`

Isso confirma dependência operacional de `Motors` e `Arming` em relação a `Battery`.

## Fluxo implícito de configuração

Ordem lógica observada no código:

1. Configurar transporte e descoberta de sensores.
2. Calibrar sensores base: IMU e barômetro.
3. Confirmar qualidade de GPS e compass antes de confiar em navegação global.
4. Selecionar backend/fonte do EKF via `AHRS_EKF_TYPE`, `EK2_*` ou `EK3_*`.
5. No caso de EKF3, definir source sets `EK3_SRCn_*` coerentes com os sensores realmente presentes.
6. Configurar `BATT_MONITOR` e limites de energia antes de depender de compensação de motores e failsafes.
7. Instanciar e ajustar controladores dependentes de AHRS e motors.
8. Mapear saídas `SERVOx_FUNCTION` coerentes com o frame e backend de motores.

## Limites desta extração

- Este seed é propositalmente focado nos subsistemas centrais pedidos: sensores principais, atuadores principais e sistemas de navegação/controle.
- A modelagem de `Battery` foi mantida no nível de frontend + parâmetros principais; não detalhei cada backend em componente separado para evitar duplicação artificial.
- Parâmetros repetitivos por instância, como `SERVOx_*` e grupos avançados de `INS_HNTC2_` a `INS_HNTC4_`, foram representados como padrões ou exemplos.
- Parâmetros repetitivos de bateria por instância (`BATT2_*`, `BATT3_*`, etc.) foram tratados como expansão do mesmo componente `Battery`.
- Dependências foram incluídas somente quando havia evidência direta em includes, construtores, agregação em classes ou famílias explícitas de parâmetros.
