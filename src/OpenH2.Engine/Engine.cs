﻿using OpenH2.Core.Architecture;
using OpenH2.Core.Configuration;
using OpenH2.Core.Extensions;
using OpenH2.Core.Factories;
using OpenH2.Engine.Components;
using OpenH2.Engine.Entities;
using OpenH2.Engine.EntityFactories;
using OpenH2.Foundation;
using OpenH2.Foundation.Engine;
using OpenH2.Physics.Colliders;
using OpenH2.Rendering.Abstractions;
using OpenH2.Rendering.OpenGL;
using System;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Numerics;

namespace OpenH2.Engine
{
    public class Engine
    {
        IGraphicsHost graphicsHost;
        IGameLoopSource gameLoop;
        

        private World world;

        public Engine()
        {
            var host = new OpenGLHost();

            graphicsHost = host;
            gameLoop = host;
        }

        public void Start(EngineStartParameters parameters)
        {
            graphicsHost.CreateWindow(new Vector2(1600, 900));

            var mapPath = parameters.LoadPathOverride ?? @"D:\H2vMaps\lockout.map";
            var configPath = Environment.GetEnvironmentVariable(ConfigurationConstants.ConfigPathOverrideEnvironmentVariable);

            if (configPath != null)
            {
                configPath = Path.GetFullPath(configPath);
            }
            else
            {
                configPath = Environment.CurrentDirectory + "/Configs";
            }

            var matFactory = new MaterialFactory(configPath);

            var factory = new MapFactory(Path.GetDirectoryName(mapPath), matFactory);

            matFactory.AddListener(() =>
            {
                LoadScene(factory, mapPath);
            });

            var rtWorld = new RealtimeWorld(this);
            rtWorld.UseGraphicsAdapter(graphicsHost.GetAdapter());

            world = rtWorld;

            LoadScene(factory, mapPath);

            gameLoop.RegisterCallbacks(world.Update, world.Render);
            gameLoop.Start(60, 60);
        }

        private void LoadScene(MapFactory factory, string mapPath)
        {
            SpectatorCamera camera = new SpectatorCamera();

            if(world.Scene != null)
            {
                camera = world.Scene.Entities.FirstOrDefault((v) => v.Value.GetType() == typeof(SpectatorCamera)).Value as SpectatorCamera;
            }

            var scene = new Scene();

            scene.AddEntity(camera);

            var watch = new Stopwatch();
            watch.Start();
            LoadMap(scene, factory, mapPath);
            watch.Stop();
            Console.WriteLine($"Loading map took {watch.ElapsedMilliseconds / 1000f} seconds");


            var floor = new Scenery();
            var floorXform = new TransformComponent(floor, Vector3.Zero);
            var floorGeom = new StaticGeometryComponent(floor)
            {
                Collider = new PlaneCollider()
                {
                    Bounds = new Physics.Bounds.AxisAlignedBoundingBox(new Vector3(-100, -100, -1), new Vector3(100, 100, 1)),
                    Distance = 1,
                    Normal = new Vector3(0, 0, 1)
                },
                Transform = floorXform
            };
            floor.SetComponents(new Component[] { floorGeom, floorXform });
            //scene.AddEntity(floor);

            world.LoadScene(scene);
        }

        public void LoadMap(Scene destination, MapFactory factory, string mapPath)
        {
            var fs = new FileStream(mapPath, FileMode.Open, FileAccess.Read, FileShare.Read, 8096);
            var map = factory.FromFile(fs);

            map.TryGetTag(map.IndexHeader.Scenario, out var scenario);

            var terrains = scenario.Terrains;

            foreach (var terrain in terrains)
            {
                map.TryGetTag(terrain.Bsp, out var bsp);

                destination.AddEntity(TerrainFactory.FromBspData(map, bsp));

                foreach(var instance in bsp.InstancedGeometryInstances)
                {
                    destination.AddEntity(SceneryFactory.FromInstancedGeometry(map, bsp, instance));
                }
            }

            foreach (var sky in scenario.SkyboxInstances)
            {
                if (sky.Skybox == uint.MaxValue)
                    continue;

                destination.AddEntity(SkyboxFactory.FromTag(map, scenario, sky));
            }

            foreach (var scen in scenario.SceneryInstances)
            {
                if (scen.SceneryDefinitionIndex == ushort.MaxValue)
                    continue;

                destination.AddEntity(SceneryFactory.FromTag(map, scenario, scen));
            }

            foreach (var bloc in scenario.BlocInstances)
            {
                destination.AddEntity(BlocFactory.FromTag(map, scenario, bloc));
            }

            foreach (var mach in scenario.MachineryInstances)
            {
                destination.AddEntity(MachineryFactory.FromTag(map, scenario, mach));
            }

            foreach (var item in scenario.ItemCollectionPlacements)
            {
                destination.AddEntity(ItemFactory.FromTag(map, scenario, item));
            }

            foreach (var item in scenario.VehicleInstances)
            {
                destination.AddEntity(ItemFactory.CreateFromVehicleInstance(map, scenario, item));
            }

            //PositioningEntities.AddLocators(map, destination);

            //PlaceLights(destination);
        }

        private void PlaceLights(Scene destination)
        {
            for(var i = 0; i < 9; i++)
            {
                var position = VectorExtensions.Random(3, 12);
                var color = VectorExtensions.RandomColor(200);

                var item = new Light();
                var model = new RenderModelComponent(item)
                {
                    RenderModel = ModelFactory.HalfTriangularThing(color)
                };

                var xform = new TransformComponent(item, position);

                var light = new PointLightEmitterComponent(item)
                {
                    Light = new PointLight()
                    {
                        Color = new Vector3(color.X, color.Y, color.Z),
                        Position = Vector3.Zero,
                        Radius = 20f
                    }
                };

                item.SetComponents(new Component[]{
                    model,
                    xform,
                    light
                });

                destination.Entities.Add(Guid.NewGuid(), item);
            }
        }
    }
}
