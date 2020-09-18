import os
import time
import yaml

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from ament_index_python.packages import get_package_share_directory

from tr_pipeline_interfaces.action import RunPipeline
from tr_pipeline_interfaces.srv import ConfigurePipeline
from tr_pipeline_interfaces.msg import PipelineType, PipelineFeedback
from composition_interfaces.srv import LoadNode, UnloadNode, ListNodes


class PipelineManager(Node):

    """
    Pipeline Manager.

    Responsible for configuring the running the pipeline
    """


    def __init__(self):
        super().__init__('pipeline_manager')

        self.nodes_in_pipeline = []
        self.pipeline_success = False
        self.pipeline_abort = False
        self.pipeline_feedback_msg = ""

        self.pipeline_types = [
            PipelineType.TYPE_CIRCLE_APPROACH,
            PipelineType.TYPE_BAR
        ]

        self.declare_parameters(
            namespace='pipeline',
            parameters=[
                ('components', ['']),
                ('remap_rules', ['']),
                ('pkg_name', ''),
                ('namespace', '')
        ])

        self.feedback_sub = self.create_subscription(
            PipelineFeedback,
            '/tr/pipeline_feedback',
            self.feedback_callback,
            10
        )

        self.pipeline_loading_client = self.create_client(
                LoadNode, 
                '/tr/pipeline/_container/load_node'
        )
        while not self.pipeline_loading_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Pipeline loading service not available, waiting again...')

        self.pipeline_unloading_client = self.create_client(
                UnloadNode, 
                '/tr/pipeline/_container/unload_node'
        )
        while not self.pipeline_unloading_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Pipeline unloading service not available, waiting again...')

        self.pipeline_listing_client = self.create_client(
                ListNodes, 
                '/tr/pipeline/_container/list_nodes'
        )
        while not self.pipeline_listing_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Pipeline listing service not available, waiting again...')

        self.pipeline_running_server = ActionServer(
            self, 
            RunPipeline, 
            'run_pipeline',
            self.run_pipeline
        )

        self.pipeline_configuring_server = self.create_service(
            ConfigurePipeline,
            'configure_pipeline',
            self.configure_pipeline
        )
            

    def feedback_callback(self, msg):
        self.pipeline_abort = msg.abort
        self.pipeline_success = msg.success
        self.pipeline_feedback_msg = msg.message


    def run_pipeline(self, goal_handle):
        self.get_logger().info('Running pipeline...')
        self.pipeline_success = False
        self.pipeline_abort = False
        self.pipeline_feedback_msg = ""
        pipeline_components = self.get_parameter('pipeline.components').value
        for component in pipeline_components:
            self._load_component(component)
        feedback = RunPipeline.Feedback()
        prev_msg = ""
        while not self.pipeline_success and not self.pipeline_abort:
            time.sleep(1)
            if len(self.pipeline_feedback_msg) > 0 and self.pipeline_feedback_msg is not prev_msg:
                # The success message may not always print due to race condition with feedback callback
                feedback.status = self.pipeline_feedback_msg
                self.get_logger().info(self.pipeline_feedback_msg)
                goal_handle.publish_feedback(feedback)
                prev_msg = self.pipeline_feedback_msg
        if self.pipeline_abort:
            goal_handle.abort()
        else: 
            goal_handle.succeed()
        self._unload_components()
        res = RunPipeline.Result()
        res.output = 0
        return res


    def configure_pipeline(self, request, response):
        pipeline_type = request.pipeline_type.type
        self.get_logger().info('Configuring pipeline for {}...'.format(pipeline_type))
        if pipeline_type not in self.pipeline_types:
            self.get_logger().warn(
                'Configuration of pipeline type "{}" is not allowed, no such type '.format(pipeline_type)
            )
            response.success = False
        else:
            manager_dir = get_package_share_directory('tr_pipeline_manager')
            config_path = os.path.join(manager_dir, '{}.yaml'.format(pipeline_type))
            config_yaml = None
            with open(config_path, 'r') as stream:
                try:
                    config_yaml = yaml.safe_load(stream)
                except yaml.YAMLError as e:
                    self.get_logger().warn('Could not parse {}.yaml'.format(pipeline_type))
                    self.get_logger().error(str(e))
            if config_yaml is not None:
                response.success = self._load_params_from_yaml(config_yaml, pipeline_type)
            else:
                response.success = False
        if response.success is True:
            self.get_logger().info('Pipeline configured for {}'.format(pipeline_type))
        return response

    
    def _load_component(self, component):
        req = LoadNode.Request()
        req.package_name = self.get_parameter('pipeline.pkg_name').value
        req.plugin_name = component
        req.node_namespace = self.get_parameter('pipeline.namespace').value
        req.remap_rules = self.get_parameter('pipeline.remap_rules').value
        future = self.pipeline_loading_client.call_async(req)
        res = None
        def callback(future):
            nonlocal res
            res = future.result()
        future.add_done_callback(callback)
        while rclpy.ok() and res is None:
            time.sleep(0.1)
        if not res.success:
            self.get_logger().warn('The component {} was not loaded succesfully'
                                    .format(component))
        else:
            node_name = res.full_node_name
            self.nodes_in_pipeline.append(node_name)
            self.get_logger().info(node_name + ' loaded succesfully')


    def _unload_components(self):
        req = ListNodes.Request()
        future = self.pipeline_listing_client.call_async(req)
        res = None
        def list_callback(future):
            nonlocal res
            res = future.result()
        future.add_done_callback(list_callback)
        while rclpy.ok() and res is None:
            time.sleep(0.1)
        for i in range(len(res.unique_ids)):
            unique_id = res.unique_ids[i]
            node_name = res.full_node_names[i]
            self._unload_component(unique_id, node_name)


    def _unload_component(self, unique_id, node_name):
        req = UnloadNode.Request()
        req.unique_id = unique_id
        future = self.pipeline_unloading_client.call_async(req)
        res = None
        def unload_callback(future):
            nonlocal res
            res = future.result()
        future.add_done_callback(unload_callback)
        while rclpy.ok() and res is None:
            time.sleep(0.1)
        if not res.success:
            self.get_logger().warn('{} was not unloaded succesfully'.format(node_name))
        else:
            self.get_logger().info('{} was unloaded succesfully'.format(node_name))


    def _load_params_from_yaml(self, config_yaml, pipeline_type):
        pipeline_params = self._get_pipeline_params(config_yaml)
        if pipeline_params is not None:
            params_list = []
            for param_name, param_val in pipeline_params.items():
                try:
                    params_list.append(self._create_param_object(param_name, param_val))
                except rclpy.exceptions.ParameterException as e:
                    self.get_logger().warn('Could not get the pipeline parameter "{}", does not exist'
                                            .format(param_name))
                    self.get_logger().error(str(e))
            try:
                self.set_parameters(params_list)
                return True
            except rclpy.exceptions.ParameterException as e:
                self.get_logger().warn('Could not set pipeline parameters')
                self.get_logger().error(str(e))
                return False
        else:
            self.get_logger().warn('Pipeline config YAML needs pipeline namespace')
            return False


    def _get_pipeline_params(self, config_yaml):
        for key, value in config_yaml.items():
            if key == "pipeline":
                return value
            elif isinstance(value, dict):
                return self._get_pipeline_params(value)
        return None

    
    def _create_param_object(self, param_name, param_val):
            param_name = 'pipeline.' + param_name
            curr_param_val = self.get_parameter(param_name).value
            param_type = rclpy.Parameter.Type.from_parameter_value(curr_param_val)
            new_param = rclpy.parameter.Parameter(
                param_name,
                param_type,
                param_val
            )
            return new_param
    

def main(args=None):
    rclpy.init(args=args)
    pipeline_manager = PipelineManager()
    executor = MultiThreadedExecutor()
    executor.add_node(pipeline_manager)
    executor.spin()
    pipeline_manager.destroy_node()
    executor.shutdown()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
