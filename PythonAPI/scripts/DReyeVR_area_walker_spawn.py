#!/usr/bin/env python3

# Copyright (c) 2025 NAIST HuRoLab
# 
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Script to track vehicle position and spawn walkers in defined area.
Based on DReyeVR_generate_traffic.py
"""

import glob
import json
import logging
import math
import os
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import argparse

import carla
from numpy import random


class WalkerSpawnConfig:
    """Configuration manager for walker spawning system"""
    
    def __init__(self, config_path: str):
        """
        Initialize configuration from JSON file
        
        Args:
            config_path: Path to configuration JSON file
        """
        self.config_path = Path(config_path)
        self._load_config()
    
    def _load_config(self) -> None:
        """Load configuration from JSON file"""
        try:
            with open(self.config_path, 'r') as f:
                self.config = json.load(f)
        except FileNotFoundError:
            raise FileNotFoundError(f"Configuration file not found: {self.config_path}")
        except json.JSONDecodeError as e:
            raise ValueError(f"Invalid JSON in configuration file: {e}")
    
    def get_simulation_config(self) -> Dict[str, Any]:
        """Get simulation configuration"""
        return self.config.get("simulation", {})
    
    def get_trigger_area(self) -> Dict[str, Any]:
        """Get trigger area configuration"""
        return self.config.get("trigger_area", {})
    
    def get_walker_spawn_config(self) -> Dict[str, Any]:
        """Get walker spawn configuration"""
        return self.config.get("walker_spawn", {})
    
    def get_vehicle_tracking_config(self) -> Dict[str, Any]:
        """Get vehicle tracking configuration"""
        return self.config.get("vehicle_tracking", {})
    
    def get_logging_config(self) -> Dict[str, Any]:
        """Get logging configuration"""
        return self.config.get("logging", {})


class VehicleTracker:
    """Tracks vehicle position and detects area entry"""
    
    def __init__(self, config: WalkerSpawnConfig):
        """
        Initialize vehicle tracker
        
        Args:
            config: Configuration manager
        """
        self.config = config
        self.trigger_area = config.get_trigger_area()
        self.vehicle_config = config.get_vehicle_tracking_config()
        
        self.center = self.trigger_area.get("center", {"x": 0, "y": 0, "z": 0})
        self.radius = self.trigger_area.get("radius", 50.0)
        self.spawn_once_per_entry = self.vehicle_config.get("spawn_once_per_entry", True)
        
        self.vehicle_in_area = False
        self.has_spawned_this_entry = False
        
        logging.info(f"Vehicle tracker initialized. Area center: ({self.center['x']}, {self.center['y']}), radius: {self.radius}")
    
    def is_vehicle_in_area(self, vehicle_location: carla.Location) -> bool:
        """
        Check if vehicle is in the trigger area
        
        Args:
            vehicle_location: Current vehicle location
            
        Returns:
            bool: True if vehicle is in area
        """
        distance = math.sqrt(
            (vehicle_location.x - self.center["x"]) ** 2 +
            (vehicle_location.y - self.center["y"]) ** 2
        )
        return distance <= self.radius
    
    def update(self, vehicle_location: carla.Location) -> bool:
        """
        Update tracker with current vehicle position
        
        Args:
            vehicle_location: Current vehicle location
            
        Returns:
            bool: True if walkers should be spawned
        """
        currently_in_area = self.is_vehicle_in_area(vehicle_location)
        
        # Detect entry into area
        if currently_in_area and not self.vehicle_in_area:
            logging.info(f"Vehicle entered trigger area at ({vehicle_location.x:.2f}, {vehicle_location.y:.2f})")
            self.vehicle_in_area = True
            
            # Spawn walkers if conditions are met
            if not self.spawn_once_per_entry or not self.has_spawned_this_entry:
                self.has_spawned_this_entry = True
                return True
        
        # Detect exit from area
        elif not currently_in_area and self.vehicle_in_area:
            logging.info(f"Vehicle exited trigger area at ({vehicle_location.x:.2f}, {vehicle_location.y:.2f})")
            self.vehicle_in_area = False
            self.has_spawned_this_entry = False
        
        return False


class WalkerSpawner:
    """Handles spawning and control of walkers"""
    
    def __init__(self, world: carla.World, config: WalkerSpawnConfig):
        """
        Initialize walker spawner
        
        Args:
            world: CARLA world instance
            config: Configuration manager
        """
        self.world = world
        self.config = config
        self.walker_config = config.get_walker_spawn_config()
        
        self.walkers_list: List[Dict[str, int]] = []
        self.all_walker_ids: List[int] = []
        
        # Get walker blueprints
        walker_filter = self.walker_config.get("walker_filter", "walker.pedestrian.*")
        generation = self.walker_config.get("generation", "2")
        self.walker_blueprints = self._get_walker_blueprints(walker_filter, generation)
        
        logging.info(f"Walker spawner initialized with {len(self.walker_blueprints)} walker blueprints")
    
    def _get_walker_blueprints(self, filter_pattern: str, generation: str) -> List[carla.ActorBlueprint]:
        """
        Get walker blueprints from world
        
        Args:
            filter_pattern: Blueprint filter pattern
            generation: Generation filter
            
        Returns:
            List of walker blueprints
        """
        bps = self.world.get_blueprint_library().filter(filter_pattern)
        
        if generation.lower() == "all":
            return list(bps)
        
        if len(bps) == 1:
            return list(bps)
        
        try:
            int_generation = int(generation)
            if int_generation in [1, 2]:
                bps = [x for x in bps if int(x.get_attribute('generation')) == int_generation]
                return list(bps)
            else:
                logging.warning("Invalid generation specified. Using all blueprints.")
                return list(bps)
        except ValueError:
            logging.warning("Invalid generation format. Using all blueprints.")
            return list(bps)
    
    def spawn_walkers(self) -> bool:
        """
        Spawn walkers at configured positions
        
        Returns:
            bool: True if spawning was successful
        """
        spawn_positions = self.walker_config.get("spawn_positions", [])
        target_positions = self.walker_config.get("target_positions", [])
        number_of_walkers = self.walker_config.get("number_of_walkers", 10)
        speed_config = self.walker_config.get("walker_speed", {})
        is_invincible = self.walker_config.get("is_invincible", False)
        
        # Limit number of walkers to available positions
        max_walkers = min(number_of_walkers, len(spawn_positions))
        
        if max_walkers == 0:
            logging.warning("No spawn positions defined. Cannot spawn walkers.")
            return False
        
        logging.info(f"Spawning {max_walkers} walkers...")
        
        # Prepare spawn points
        spawn_points = []
        walker_speeds = []
        
        for i in range(max_walkers):
            pos_idx = i % len(spawn_positions)
            spawn_pos = spawn_positions[pos_idx]
            
            spawn_point = carla.Transform()
            spawn_point.location = carla.Location(
                x=spawn_pos["x"],
                y=spawn_pos["y"],
                z=spawn_pos["z"]
            )
            spawn_points.append(spawn_point)
            
            # Determine walker speed
            min_speed = speed_config.get("min", 1.0)
            max_speed = speed_config.get("max", 2.5)
            default_speed = speed_config.get("default", 1.5)
            
            if min_speed == max_speed:
                speed = default_speed
            else:
                speed = random.uniform(min_speed, max_speed)
            walker_speeds.append(speed)
        
        # Spawn walker actors
        batch = []
        for i, spawn_point in enumerate(spawn_points):
            walker_bp = random.choice(self.walker_blueprints)
            
            # Set invincibility
            if walker_bp.has_attribute('is_invincible'):
                walker_bp.set_attribute('is_invincible', str(is_invincible).lower())
            
            batch.append(carla.command.SpawnActor(walker_bp, spawn_point))
        
        # Execute spawn batch
        client = self.world.get_client()
        results = client.apply_batch_sync(batch, True)
        
        # Process spawn results
        successful_spawns = []
        for i, result in enumerate(results):
            if result.error:
                logging.error(f"Failed to spawn walker {i}: {result.error}")
            else:
                self.walkers_list.append({"id": result.actor_id})
                successful_spawns.append(i)
        
        if not successful_spawns:
            logging.error("Failed to spawn any walkers")
            return False
        
        # Spawn walker controllers
        batch = []
        walker_controller_bp = self.world.get_blueprint_library().find('controller.ai.walker')
        for walker_data in self.walkers_list:
            batch.append(carla.command.SpawnActor(walker_controller_bp, carla.Transform(), walker_data["id"]))
        
        results = client.apply_batch_sync(batch, True)
        
        # Process controller spawn results
        for i, result in enumerate(results):
            if result.error:
                logging.error(f"Failed to spawn walker controller {i}: {result.error}")
            else:
                self.walkers_list[i]["con"] = result.actor_id
        
        # Build ID list for actor retrieval
        for walker_data in self.walkers_list:
            self.all_walker_ids.append(walker_data["con"])
            self.all_walker_ids.append(walker_data["id"])
        
        # Get all actors
        all_actors = self.world.get_actors(self.all_walker_ids)
        
        # Wait for tick to ensure actors are ready
        self.world.wait_for_tick()
        
        # Initialize walker controllers and set targets
        for i in range(0, len(self.all_walker_ids), 2):
            controller_actor = all_actors[i]
            walker_idx = i // 2
            
            # Start walker
            controller_actor.start()
            
            # Set target location
            if walker_idx < len(target_positions):
                target_pos = target_positions[walker_idx % len(target_positions)]
                target_location = carla.Location(
                    x=target_pos["x"],
                    y=target_pos["y"],
                    z=target_pos["z"]
                )
                controller_actor.go_to_location(target_location)
            else:
                # Use random location if no specific target
                controller_actor.go_to_location(self.world.get_random_location_from_navigation())
            
            # Set speed
            if walker_idx < len(walker_speeds):
                controller_actor.set_max_speed(walker_speeds[walker_idx])
        
        logging.info(f"Successfully spawned {len(self.walkers_list)} walkers with controllers")
        return True
    
    def destroy_walkers(self) -> None:
        """Destroy all spawned walkers and controllers"""
        if not self.all_walker_ids:
            return
        
        # Stop controllers
        all_actors = self.world.get_actors(self.all_walker_ids)
        for i in range(0, len(self.all_walker_ids), 2):
            try:
                all_actors[i].stop()
            except Exception as e:
                logging.warning(f"Failed to stop walker controller: {e}")
        
        # Destroy all actors
        client = self.world.get_client()
        client.apply_batch([carla.command.DestroyActor(x) for x in self.all_walker_ids])
        
        logging.info(f"Destroyed {len(self.walkers_list)} walkers")
        
        # Clear lists
        self.walkers_list.clear()
        self.all_walker_ids.clear()


def find_hero_vehicle(world: carla.World, role_name: str = "hero") -> Optional[carla.Vehicle]:
    """
    Find the hero vehicle in the world
    
    Args:
        world: CARLA world instance
        role_name: Role name to search for
        
    Returns:
        Hero vehicle actor or None if not found
    """
    for actor in world.get_actors():
        if hasattr(actor, 'attributes') and actor.attributes.get('role_name') == role_name:
            if 'vehicle' in actor.type_id:
                return actor
    return None


def main():
    """Main function"""
    # Parse command line arguments
    argparser = argparse.ArgumentParser(description=__doc__)
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)'
    )
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)'
    )
    argparser.add_argument(
        '--config',
        default='walker_spawn_config.json',
        help='Path to configuration file (default: walker_spawn_config.json)'
    )
    argparser.add_argument(
        '--log-level',
        default='INFO',
        choices=['DEBUG', 'INFO', 'WARNING', 'ERROR'],
        help='Logging level (default: INFO)'
    )
    
    args = argparser.parse_args()
    
    # Setup logging
    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format='%(asctime)s - %(levelname)s - %(message)s'
    )
    
    # Load configuration
    try:
        config = WalkerSpawnConfig(args.config)
    except (FileNotFoundError, ValueError) as e:
        logging.error(f"Configuration error: {e}")
        return 1
    
    # Connect to CARLA
    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    
    try:
        world = client.get_world()
        
        # Configure simulation settings
        sim_config = config.get_simulation_config()
        settings = world.get_settings()
        settings.synchronous_mode = sim_config.get("sync_mode", False)
        if not settings.synchronous_mode:
            settings.fixed_delta_seconds = None
        else:
            settings.fixed_delta_seconds = sim_config.get("delta_t", 0.05)
        world.apply_settings(settings)
        
        logging.info(f"Simulation configured - Sync mode: {settings.synchronous_mode}")
        
        # Initialize components
        vehicle_tracker = VehicleTracker(config)
        walker_spawner = WalkerSpawner(world, config)
        
        # Get tracking configuration
        vehicle_config = config.get_vehicle_tracking_config()
        role_name = vehicle_config.get("role_name", "hero")
        check_interval = vehicle_config.get("check_interval", 0.1)
        
        logging.info(f"Starting vehicle tracking for role: {role_name}")
        
        # Main loop
        last_check_time = 0
        while True:
            current_time = time.time()
            
            # Check at specified interval
            if current_time - last_check_time >= check_interval:
                # Find hero vehicle
                hero_vehicle = find_hero_vehicle(world, role_name)
                
                if hero_vehicle is None:
                    logging.debug(f"Hero vehicle with role '{role_name}' not found")
                else:
                    # Get vehicle location
                    vehicle_location = hero_vehicle.get_location()
                    
                    # Update tracker and check if walkers should be spawned
                    should_spawn = vehicle_tracker.update(vehicle_location)
                    
                    if should_spawn:
                        success = walker_spawner.spawn_walkers()
                        if success:
                            logging.info("Walkers spawned successfully")
                        else:
                            logging.error("Failed to spawn walkers")
                
                last_check_time = current_time
            
            # Handle world tick based on sync mode
            if settings.synchronous_mode:
                world.tick()
            else:
                time.sleep(0.01)  # Small sleep to prevent high CPU usage
    
    except KeyboardInterrupt:
        logging.info("Interrupted by user")
    except Exception as e:
        logging.error(f"Unexpected error: {e}")
        return 1
    finally:
        # Cleanup
        try:
            walker_spawner.destroy_walkers()
        except Exception as e:
            logging.warning(f"Error during cleanup: {e}")
        
        # Reset world settings
        try:
            settings = world.get_settings()
            settings.synchronous_mode = False
            settings.fixed_delta_seconds = None
            world.apply_settings(settings)
        except Exception as e:
            logging.warning(f"Failed to reset world settings: {e}")
    
    return 0


if __name__ == '__main__':
    try:
        exit_code = main()
        sys.exit(exit_code)
    except KeyboardInterrupt:
        pass
    finally:
        logging.info("Script finished")
