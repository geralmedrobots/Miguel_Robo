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

# ============================================================================
# CRITICAL: Generate Safety Parameter Certification Files
# ============================================================================
# The SafetySupervisor node requires TWO certification files to start:
# 1. config/certified_safety_params.yaml (already in repo)
# 2. config/cert.key (HMAC secret key - NOT in repo for security)
#
# For DEVELOPMENT/TESTING environments:
cd ~/ultrabot_ws/src/navigation  # or your workspace path

# Generate cert.key with development secret
python3 scripts/generate_certification_hash.py

# This creates:
# - config/cert.key (HMAC secret - DO NOT commit to git!)
# - Updates certified_safety_params.yaml with new hash
#
# For PRODUCTION/DEPLOYMENT:
# 1. Generate cert.key on secure build server
# 2. Deploy cert.key via encrypted config management (Vault, Ansible Vault, etc.)
# 3. NEVER commit cert.key to version control
# 4. Rotate cert.key periodically (recommended: every 6 months)
#
# Without these files, the system will fail with:
# "DEPLOYMENT ERROR: cert.key NOT FOUND!"
# ============================================================================

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
# /cmd_vel_safe
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

---

## 🚨 Deployment Troubleshooting

### Error: "DEPLOYMENT ERROR: cert.key NOT FOUND!"

**Causa:** O ficheiro `config/cert.key` não existe no sistema de destino.

**Solução:**
```bash
# Development/Testing:
cd ~/ultrabot_ws/src/navigation
python3 scripts/generate_certification_hash.py

# Production:
# 1. Generate cert.key on secure build server
# 2. Deploy via encrypted configuration management:
#    - Ansible Vault: ansible-vault encrypt cert.key
#    - HashiCorp Vault: vault kv put secret/ultrabot cert_key=@cert.key
#    - Kubernetes Secret: kubectl create secret generic ultrabot-cert --from-file=cert.key
# 3. Never commit cert.key to git (already in .gitignore)

# Zero-touch deployments (CI/CD):
# - Encode the secret and inject as environment variable
export SAFETY_SUPERVISOR_CERT_KEY_B64=$(base64 -w0 config/cert.key)

# - Or point to an external secret file managed by the host OS
export SAFETY_SUPERVISOR_CERT_KEY_PATH=/etc/ultrabot/cert.key
```

> 💡 **Dica:** também pode definir o parâmetro ROS `cert_key_path_override`
> (no `SafetySupervisor`) para apontar diretamente para um ficheiro de
> segredo gerido fora do workspace.

**Verificação:**
```bash
# Check if cert.key exists
ls -l ~/ultrabot_ws/install/somanet/share/somanet/config/cert.key

# Expected: File should exist with permissions 600 or 400
# If missing: Run generate_certification_hash.py or set SAFETY_SUPERVISOR_CERT_KEY_B64
```

### Error: "Certified parameters validation FAILED!"

**Causa:** O hash HMAC em `certified_safety_params.yaml` não corresponde ao conteúdo + `cert.key`.

**Sintomas:**
- "Possible tampering detected"
- "Verify cert.key matches certified_safety_params.yaml"

**Solução:**
```bash
# Regenerate certification (development only)
cd ~/ultrabot_ws/src/navigation
python3 scripts/generate_certification_hash.py

# For production: Contact safety officer
# - Parameter changes require safety review
# - New certification must be documented
# - Audit trail must be updated
```

**Verificação:**
```bash
# Manually verify hash (development)
python3 -c "
import hashlib
import yaml

# Load certified params
with open('config/certified_safety_params.yaml', 'r') as f:
    data = yaml.safe_load(f)

# Load secret key
with open('config/cert.key', 'r') as f:
    secret = f.read().strip()

# Compute HMAC
import hmac
params = data['safety_limits']
param_str = ';'.join(f'{k}={v["value"]:.6f}' for k, v in sorted(params.items())) + ';'
computed_hmac = hmac.new(secret.encode(), param_str.encode(), hashlib.sha256).hexdigest()

print(f'Stored HMAC:   {data["certification"]["hmac"]}')
print(f'Computed HMAC: {computed_hmac}')
print(f'Match: {computed_hmac == data["certification"]["hmac"]}')
"
```

### Error: "Failed to find package 'somanet'"

**Causa:** Workspace não foi sourced ou package não foi compilado.

**Solução:**
```bash
# 1. Build package
cd ~/ultrabot_ws
colcon build --packages-select somanet

# 2. Source workspace
source install/setup.bash

# 3. Verify
ros2 pkg prefix somanet
```

### Production Deployment Checklist

**Before deploying to production:**

- [ ] `cert.key` generated on secure build server
- [ ] `cert.key` deployed via encrypted config management (NOT git)
- [ ] `certified_safety_params.yaml` hash matches cert.key
- [ ] Certificate not expired (check `valid_until` field)
- [ ] Safety parameters reviewed and approved
- [ ] Audit trail updated in certified_safety_params.yaml
- [ ] cert.key file permissions set to 400 (read-only)
- [ ] Backup of cert.key stored in secure vault
- [ ] Deployment documented in change log

**Verification commands:**
```bash
# 1. Check cert.key exists and has correct permissions
ls -l install/somanet/share/somanet/config/cert.key
# Expected: -r-------- 1 user group ... cert.key

# 2. Verify certification metadata
grep -A 5 "certification:" install/somanet/share/somanet/config/certified_safety_params.yaml

# 3. Test safety supervisor startup
ros2 run somanet safety_supervisor_node --ros-args --log-level info
# Expected: "✅ Certified parameters loaded (Cert: ULTRABOT-..., Valid: ...)"

# 4. Verify no tampering warnings
ros2 topic echo /diagnostics | grep -i tamper
# Expected: No output (no tampering detected)
```

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
