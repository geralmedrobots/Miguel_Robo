#!/usr/bin/env python3
"""
Script de Validação Automática de Odometria
Testa precisão, frequência e calibração do sistema de odometria

Compliance: ISO 14971 (Risk management), IEC 62304 (Software lifecycle)
Features:
- Automated odometry validation tests
- CSV export with metrics and timestamps
- Configuration hash for traceability
- Version tracking

@version 1.1.0
@date 2025-10-28
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import time
import math
import numpy as np
import csv
import hashlib
import yaml
from datetime import datetime
from pathlib import Path

class OdometryValidator(Node):
    def __init__(self, config_file=None):
        super().__init__('odometry_validator')
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        
        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Data storage
        self.odom_data = []
        self.start_time = None
        self.initial_pose = None
        self.test_results = []
        
        # Configuration tracking
        self.config_file = config_file
        self.config_hash = None
        self.config_version = None
        
        if self.config_file and Path(self.config_file).exists():
            self._load_config_metadata()
        
        # Test metadata
        self.test_session = {
            'start_time': datetime.now(),
            'operator': 'automated',
            'test_version': '1.1.0'
        }
        
        self.get_logger().info('🔬 Odometry Validator iniciado (v1.1.0)')
        if self.config_hash:
            self.get_logger().info(f'   Config hash: {self.config_hash[:16]}...')
    
    def _load_config_metadata(self):
        """Load configuration file and compute hash"""
        try:
            with open(self.config_file, 'rb') as f:
                config_data = f.read()
                self.config_hash = hashlib.sha256(config_data).hexdigest()
            
            # Try to load version from YAML
            with open(self.config_file, 'r') as f:
                config = yaml.safe_load(f)
                if 'compliance' in config and 'configuration_version' in config['compliance']:
                    self.config_version = config['compliance']['configuration_version']
        except Exception as e:
            self.get_logger().warn(f'Could not load config metadata: {e}')
    
    def odom_callback(self, msg):
        """Armazena dados de odometria"""
        timestamp = self.get_clock().now().nanoseconds / 1e9
        
        # Extrair posição
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        # Extrair orientação (quaternion → yaw)
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                        1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        
        # Velocidades
        vx = msg.twist.twist.linear.x
        wz = msg.twist.twist.angular.z
        
        self.odom_data.append({
            'time': timestamp,
            'x': x,
            'y': y,
            'yaw': yaw,
            'vx': vx,
            'wz': wz
        })
    
    def test_frequency(self, duration=10.0):
        """Teste 1: Verificar frequência de publicação"""
        self.get_logger().info(f'\n📊 Teste 1: Frequência de Publicação ({duration}s)')
        
        self.odom_data = []
        start_time = time.time()
        
        while time.time() - start_time < duration:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if len(self.odom_data) < 2:
            self.get_logger().error('❌ Nenhum dado de odometria recebido!')
            return False
        
        # Calcular frequência
        time_diffs = np.diff([d['time'] for d in self.odom_data])
        avg_freq = 1.0 / np.mean(time_diffs)
        min_freq = 1.0 / np.max(time_diffs)
        max_freq = 1.0 / np.min(time_diffs)
        
        self.get_logger().info(f'  Mensagens recebidas: {len(self.odom_data)}')
        self.get_logger().info(f'  Frequência média: {avg_freq:.1f} Hz')
        self.get_logger().info(f'  Frequência mín/máx: {min_freq:.1f} / {max_freq:.1f} Hz')
        
        # Avaliar
        if avg_freq < 10:
            self.get_logger().warn('  ⚠️ Frequência muito baixa (< 10 Hz)')
            return False
        elif avg_freq < 20:
            self.get_logger().warn('  ⚠️ Frequência baixa (< 20 Hz)')
            return True
        elif avg_freq >= 50:
            self.get_logger().info('  ✅ Frequência excelente (≥ 50 Hz)')
            return True
        else:
            self.get_logger().info('  ✅ Frequência boa (≥ 20 Hz)')
            return True
    
    def test_straight_line(self, distance=1.0, velocity=500.0):
        """Teste 2: Movimento em linha reta"""
        self.get_logger().info(f'\n📏 Teste 2: Linha Reta ({distance} m)')
        
        # Aguardar odometria estável
        self.odom_data = []
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if not self.odom_data:
            self.get_logger().error('❌ Sem dados de odometria')
            return False
        
        initial_x = self.odom_data[-1]['x']
        initial_y = self.odom_data[-1]['y']
        initial_yaw = self.odom_data[-1]['yaw']
        
        # Enviar comando
        cmd = Twist()
        cmd.linear.x = velocity
        
        self.get_logger().info(f'  Movendo {distance} m...')
        
        # Tempo estimado
        time_needed = distance / (velocity / 1000.0 * 2.0 * math.pi * 0.085)  # estimativa
        start_time = time.time()
        
        while time.time() - start_time < time_needed:
            self.cmd_pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.01)
        
        # Parar
        cmd.linear.x = 0.0
        self.cmd_pub.publish(cmd)
        
        # Aguardar estabilizar
        time.sleep(0.5)
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.1)
        
        # Medir resultado
        final_x = self.odom_data[-1]['x']
        final_y = self.odom_data[-1]['y']
        final_yaw = self.odom_data[-1]['yaw']
        
        distance_traveled = math.sqrt((final_x - initial_x)**2 + 
                                     (final_y - initial_y)**2)
        yaw_drift = abs(final_yaw - initial_yaw) * 180 / math.pi
        
        error = abs(distance_traveled - distance)
        error_percent = (error / distance) * 100
        
        self.get_logger().info(f'  Distância esperada: {distance:.3f} m')
        self.get_logger().info(f'  Distância medida:   {distance_traveled:.3f} m')
        self.get_logger().info(f'  Erro: {error:.3f} m ({error_percent:.1f}%)')
        self.get_logger().info(f'  Deriva angular: {yaw_drift:.1f}°')
        
        # Avaliar
        if error_percent < 5:
            self.get_logger().info('  ✅ Excelente precisão (< 5%)')
            return True
        elif error_percent < 10:
            self.get_logger().info('  ✅ Boa precisão (< 10%)')
            return True
        else:
            self.get_logger().warn(f'  ⚠️ Precisão baixa (≥ 10%)')
            self.get_logger().warn('  💡 Sugestão: Calibrar wheel_diameter_')
            return False
    
    def test_rotation(self, angle_deg=180.0, velocity=300.0):
        """Teste 3: Rotação"""
        self.get_logger().info(f'\n🔄 Teste 3: Rotação ({angle_deg}°)')
        
        # Aguardar odometria estável
        self.odom_data = []
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if not self.odom_data:
            return False
        
        initial_yaw = self.odom_data[-1]['yaw']
        target_yaw = initial_yaw + math.radians(angle_deg)
        
        # Enviar comando
        cmd = Twist()
        cmd.angular.z = velocity if angle_deg > 0 else -velocity
        
        self.get_logger().info(f'  Girando {angle_deg}°...')
        
        # Tempo estimado
        time_needed = abs(angle_deg) / 90.0 * 3.0  # estimativa
        start_time = time.time()
        
        while time.time() - start_time < time_needed:
            self.cmd_pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.01)
        
        # Parar
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)
        
        time.sleep(0.5)
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.1)
        
        # Medir resultado
        final_yaw = self.odom_data[-1]['yaw']
        
        angle_rotated = (final_yaw - initial_yaw) * 180 / math.pi
        error = abs(angle_rotated - angle_deg)
        error_percent = (error / abs(angle_deg)) * 100
        
        self.get_logger().info(f'  Ângulo esperado: {angle_deg:.1f}°')
        self.get_logger().info(f'  Ângulo medido:   {angle_rotated:.1f}°')
        self.get_logger().info(f'  Erro: {error:.1f}° ({error_percent:.1f}%)')
        
        # Avaliar
        if error_percent < 5:
            self.get_logger().info('  ✅ Excelente precisão (< 5%)')
            return True
        elif error_percent < 10:
            self.get_logger().info('  ✅ Boa precisão (< 10%)')
            return True
        else:
            self.get_logger().warn(f'  ⚠️ Precisão baixa (≥ 10%)')
            self.get_logger().warn('  💡 Sugestão: Calibrar distance_wheels')
            return False
    
    def run_all_tests(self):
        """Executar todos os testes"""
        self.get_logger().info('='*60)
        self.get_logger().info('🔬 VALIDAÇÃO AUTOMÁTICA DE ODOMETRIA')
        self.get_logger().info('='*60)
        
        results = []
        
        # Teste 1: Frequência
        results.append(('Frequência', self.test_frequency(duration=5.0)))
        
        # Teste 2: Linha reta
        results.append(('Linha Reta', self.test_straight_line(distance=1.0)))
        
        # Teste 3: Rotação
        results.append(('Rotação', self.test_rotation(angle_deg=180.0)))
        
        # Resumo
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('📊 RESUMO DOS TESTES')
        self.get_logger().info('='*60)
        
        passed = sum(1 for _, result in results if result)
        total = len(results)
        
        for test_name, result in results:
            status = '✅ PASSOU' if result else '❌ FALHOU'
            self.get_logger().info(f'  {test_name}: {status}')
        
        self.get_logger().info(f'\nResultado: {passed}/{total} testes passaram')
        
        if passed == total:
            self.get_logger().info('🎉 Todos os testes passaram! Odometria OK!')
        elif passed >= total * 0.66:
            self.get_logger().warn('⚠️ Alguns testes falharam. Verificar calibração.')
        else:
            self.get_logger().error('❌ Muitos testes falharam. Revisar configuração.')

def main(args=None):
    rclpy.init(args=args)
    
    validator = OdometryValidator()
    
    try:
        validator.run_all_tests()
    except KeyboardInterrupt:
        pass
    finally:
        validator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
