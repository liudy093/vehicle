import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
AUTODRIVE_WORKFLOW = ROOT / "deploy" / "autodrive" / "argo" / "vehicle-autodrive-workflow.yaml"
MAPPING_WORKFLOW = ROOT / "deploy" / "mapping" / "argo" / "vehicle-mapping-workflow.yaml"
JOIN_SCRIPT = ROOT / "deploy" / "shared" / "k8s" / "join_vehicle_k3s_agent.sh"
GENERAL_WORKERS_SCRIPT = ROOT / "deploy" / "shared" / "k8s" / "register_general_workers.sh"
MAPPING_CONFIGMAP = ROOT / "deploy" / "shared" / "k8s" / "mapping-cloud-configmap.yaml"
MAPPING_DEPLOYMENT = ROOT / "deploy" / "shared" / "k8s" / "mapping-cloud-deployment.yaml"
MAPPING_SERVICE = ROOT / "deploy" / "shared" / "k8s" / "mapping-cloud-service.yaml"
MAPPING_NAMESPACE = ROOT / "deploy" / "shared" / "k8s" / "namespace-mapping.yaml"
VEHICLE_NAMESPACE = ROOT / "deploy" / "shared" / "k8s" / "namespace-vehicle.yaml"
ARGO_INSTALL_SCRIPT = ROOT / "deploy" / "shared" / "k8s" / "argo-install.sh"


class VehicleNamingMigrationTests(unittest.TestCase):
    def test_shared_scripts_use_vehicle_node_contract(self) -> None:
        join_text = JOIN_SCRIPT.read_text()
        worker_text = GENERAL_WORKERS_SCRIPT.read_text()

        self.assertIn("vehicle.role=vehicle", join_text)
        self.assertIn("vehicle.id=${VEHICLE_ID}", join_text)
        self.assertIn("vehicle.role=vehicle:NoSchedule", join_text)
        self.assertIn("vehicle.role=general", worker_text)
        self.assertNotIn("perception.role", join_text)
        self.assertNotIn("perception.vehicle.id", join_text)
        self.assertNotIn("perception.role", worker_text)

    def test_workflows_do_not_reference_perception_contracts(self) -> None:
        autodrive_text = AUTODRIVE_WORKFLOW.read_text()
        mapping_text = MAPPING_WORKFLOW.read_text()

        self.assertIn("vehicle.id:", autodrive_text)
        self.assertIn("vehicle.role", autodrive_text)
        self.assertIn("vehicle.id:", mapping_text)
        self.assertIn("vehicle.role", mapping_text)
        self.assertNotIn("perception.vehicle.id", autodrive_text)
        self.assertNotIn("perception.role", autodrive_text)
        self.assertNotIn("perception.vehicle.id", mapping_text)
        self.assertNotIn("perception.role", mapping_text)

    def test_mapping_manifests_use_mapping_names(self) -> None:
        self.assertTrue(MAPPING_CONFIGMAP.exists())
        self.assertTrue(MAPPING_DEPLOYMENT.exists())
        self.assertTrue(MAPPING_SERVICE.exists())
        self.assertFalse(MAPPING_NAMESPACE.exists())
        self.assertFalse(VEHICLE_NAMESPACE.exists())

        configmap_text = MAPPING_CONFIGMAP.read_text()
        deployment_text = MAPPING_DEPLOYMENT.read_text()
        service_text = MAPPING_SERVICE.read_text()
        argo_install_text = ARGO_INSTALL_SCRIPT.read_text()

        self.assertIn("name: mapping-cloud-config", configmap_text)
        self.assertIn("namespace: argo", configmap_text)
        self.assertIn('GLOBAL_MAP_EXPORT_DIR: "/tmp/mapping_maps"', configmap_text)
        self.assertIn("name: mapping-cloud", deployment_text)
        self.assertIn("namespace: argo", deployment_text)
        self.assertIn("app: mapping-cloud", deployment_text)
        self.assertIn("name: mapping-cloud", service_text)
        self.assertIn("namespace: argo", service_text)
        self.assertIn("app.kubernetes.io/name: mapping-cloud", service_text)
        self.assertIn('ARGO_NAMESPACE="${ARGO_NAMESPACE:-argo}"', argo_install_text)
        self.assertNotIn("MAPPING_NAMESPACE", argo_install_text)
        self.assertNotIn("VEHICLE_NAMESPACE", argo_install_text)
        self.assertNotIn("perception-cloud", configmap_text)
        self.assertNotIn("perception-cloud", deployment_text)
        self.assertNotIn("perception-cloud", service_text)

    def test_mapping_workflow_uses_argo_namespace_for_cloud_service(self) -> None:
        mapping_text = MAPPING_WORKFLOW.read_text()

        self.assertIn("deployment/mapping-cloud -n argo", mapping_text)
        self.assertIn("mapping-cloud.argo.svc.cluster.local", mapping_text)
        self.assertNotIn("mapping-cloud.mapping.svc.cluster.local", mapping_text)


if __name__ == "__main__":
    unittest.main()
