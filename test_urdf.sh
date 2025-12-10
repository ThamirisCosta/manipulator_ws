#!/bin/bash
# Script de teste para verificar qual URDF está sendo usado

echo "=== Verificando URDF Real ==="
echo ""
echo "Plugin no URDF Real:"
grep -A 3 "<plugin>" src/manipulator_description_pkg/urdf/manipulator_description_real.urdf | head -10
echo ""
echo "=== Verificando URDF de Simulação ==="
echo ""
echo "Plugin no URDF de Simulação:"
grep -A 3 "<plugin>" src/manipulator_description_pkg/urdf/manipulator_description.urdf | head -10
