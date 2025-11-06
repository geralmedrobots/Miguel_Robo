# 🚀 GUIA RÁPIDO DE COMPILAÇÃO E TESTE

## 📋 Pré-requisitos

```bash
# Verificar ROS 2 Humble
source /opt/ros/humble/setup.bash
ros2 --version

# Instalar dependências
sudo apt-get update
sudo apt-get install -y \
    ros-humble-diagnostic-msgs \
    ros-humble-sros2 \
    python3-yaml \
    python3-numpy

# Criar o arquivo com a chave secreta HMAC para desenvolvimento
# Em produção, este arquivo deve ser gerenciado por um sistema de segredos (ex: Vault)
echo "ThisIsAPlaceholderSecretKeyForDevelopment!ChangeMe!" > src/navigation/config/cert.key

# Opcional: Configurar SROS2 (comunicação criptografada)
# Recomendado para produção - Ver SROS2_GUIDE.md para detalhes
cd src/navigation/scripts
./setup_sros2.sh
```

## 🔨 Compilação

```bash
# Navegar para o workspace
cd ~/ultrabot_ws  # ou caminho do seu workspace

# Compilar o pacote
colcon build --packages-select somanet

# Source do workspace
source install/setup.bash
```

### ⚠️ Erros Comuns de Compilação

**Erro:** `fatal error: algorithm: No such file or directory`

**Solução:** Adicione `#include <algorithm>` no `teleop_joy.cpp`

**Erro:** `diagnostic_msgs not found`

**Solução:**
```bash
sudo apt-get install ros-humble-diagnostic-msgs
```

## ✅ Verificação da Instalação

```bash
# Listar executáveis disponíveis
ros2 pkg executables somanet

# Deve mostrar:
# somanet main
# somanet teleop_joy
# somanet safety_supervisor_node
# somanet teleop_keyboard.py
# somanet teleop_keyboard_safe.py
# somanet validate_odometry.py
```

## 🧪 Testes Básicos (Sem Hardware)

### Teste 1: Safety Supervisor

```bash
# Terminal 1: Launch safety supervisor
ros2 run somanet safety_supervisor_node \
  --ros-args \
  --params-file src/navigation/config/safety_params.yaml

# Deve mostrar:
# [INFO] [safety_supervisor]: Safety Supervisor initialized
# [INFO] [safety_supervisor]: Limits: linear=1.00 m/s, angular=1.00 rad/s
```

### Teste 2: Diagnostics

```bash
# Terminal 2: Monitorar diagnósticos
ros2 topic echo /diagnostics

# Deve mostrar atualizações a cada 1 segundo
```

### Teste 3: Teleop Teclado Seguro

```bash
# Terminal 3: Launch teleop keyboard
ros2 run somanet teleop_keyboard_safe.py

# Teste:
# 1. Pressione SPACE (deadman)
# 2. Pressione 'i' (forward)
# 3. Verifique comandos em /cmd_vel
```

### Teste 4: Verificar Tópicos

```bash
# Listar todos os tópicos
ros2 topic list

# Deve incluir:
# /cmd_vel
# /wheel_cmd_safe
# /safety_stop
# /deadman_status
# /diagnostics
# /operator_log
```

## 🎮 Teste com Joystick (Se disponível)

```bash
# Terminal 1: Joy node
ros2 run joy joy_node

# Terminal 2: Safety supervisor
ros2 run somanet safety_supervisor_node --autostart \
  --ros-args --params-file config/safety_params.yaml

# Terminal 3: Teleop joy
ros2 run somanet teleop_joy \
  --ros-args --params-file config/safety_params.yaml

# Teste:
# 1. Segure R1 (deadman)
# 2. Pressione R2 (throttle)
# 3. Mova os sticks
# 4. Verifique comandos em /cmd_vel
```

> Nota: Nós críticos (`safety_supervisor`, `somanet_driver`, `command_arbitrator`) iniciam em estado `unconfigured`. Use `--autostart`, defina `ULTRABOT_AUTOSTART=1` antes de executar, ou dispare as transições com `ros2 lifecycle set <node> configure` / `activate`.

## 🔍 Debug de Problemas

### Ver logs detalhados

```bash
# Launch com nível de log DEBUG
ros2 run somanet safety_supervisor_node --ros-args --log-level debug
```

### Verificar comunicação

```bash
# Ver frequência dos tópicos
ros2 topic hz /cmd_vel
ros2 topic hz /diagnostics

# Ver conteúdo dos tópicos
ros2 topic echo /safety_stop
ros2 topic echo /deadman_status
```

### Testar Safety Stop

```bash
# Publicar safety stop manualmente
ros2 topic pub /safety_stop std_msgs/Bool "data: true"

# Robot deve parar imediatamente
# Verifique logs do safety supervisor
```

## 📊 Validação de Odometria (Com Hardware)

```bash
# Com robot ligado e odometria funcionando
ros2 run somanet validate_odometry.py

# Testes executados:
# 1. Frequência de publicação
# 2. Precisão em linha reta
# 3. Precisão em rotação

# Resultados devem ser > 90% para aprovação
```

## 🛠️ Troubleshooting

### Problema: "Permission denied" no Safety Supervisor

**Causa:** Falta de permissões para raw sockets

**Solução:**
```bash
sudo usermod -aG realtime $USER
# Reboot necessário
```

### Problema: Watchdog timeout imediato

**Causa:** Nenhum comando sendo enviado

**Solução:**
- Certifique-se que teleop node está rodando
- Verifique se joystick está conectado
- Teste com teleop keyboard

### Problema: Plausibility check falha

**Causa:** Odometria não disponível ou valores incorretos

**Solução:**
```bash
# Verificar se odometria está publicando
ros2 topic hz /odom

# Se não estiver, lance o drive node
sudo ros2 run somanet main --autostart
```

## 📝 Checklist de Validação

Antes de operar com hardware:

- [ ] Compilação sem erros
- [ ] Safety supervisor inicia corretamente
- [ ] Diagnósticos publicam a 1 Hz
- [ ] Teleop responde ao deadman
- [ ] Comandos são validados corretamente
- [ ] Safety stop funciona
- [ ] Watchdog timeout detectado
- [ ] Logs de operador funcionam

## 🔒 Segurança em Produção (Opcional mas Recomendado)

Para ambientes de produção, ative SROS2 para criptografar toda comunicação:

```bash
# 1. Gerar certificados de segurança
cd src/navigation/scripts
./setup_sros2.sh

# 2. Ativar SROS2 (adicionar ao ~/.bashrc para permanente)
export ROS_SECURITY_KEYSTORE=~/ultrabot_ws/src/navigation/sros2_keystore
export ROS_SECURITY_ENABLE=true
export ROS_SECURITY_STRATEGY=Enforce

# 3. Lançar normalmente - a segurança é automática
ros2 launch somanet launch.py
```

**Benefícios do SROS2:**
- ✅ Comunicação criptografada (AES-256)
- ✅ Autenticação mútua entre nós
- ✅ Controle de acesso granular
- ✅ Conformidade com IEC 62443

**Documentação completa:** Ver [SROS2_GUIDE.md](SROS2_GUIDE.md)

---

## 🎯 Próximo Passo: Teste com Hardware

Quando todos os testes acima passarem, consulte:
- **SAFETY.md** para procedimentos de operação segura
- **README.md** para instruções completas
- **SROS2_GUIDE.md** para segurança de comunicação

---

**⚠️ ATENÇÃO:** Nunca opere o hardware sem:
1. Ler completamente SAFETY.md
2. Completar checklist de segurança
3. Ter supervisor presente (se mandatório)
4. Testar emergency stop físico
5. **PRODUÇÃO:** Ativar SROS2 para comunicação segura

---

**Boa sorte com os testes! 🚀**
