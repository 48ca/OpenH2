﻿using CommandLine;
using OpenH2.Core.Extensions;
using OpenH2.Core.Factories;
using OpenH2.Core.Maps;
using OpenH2.Core.Patching;
using OpenH2.Serialization;
using System.IO;
using System.Text.Json;
using System.Threading.Tasks;

namespace OpenH2.MccUtil
{
    [Verb("patch-tag")]
    public class PatchTagCommandLineArguments
    {
        [Option('p', "patch", Required = true, HelpText = "The tag patch to load")]
        public string TagPatchPath { get; set; }

        [Option('m', "map", HelpText = "The map to apply the patch to")]
        public string MapPath { get; set; }
    }


    public class PatchTagTask
    {
        private H2BaseMap scene;

        public PatchTagCommandLineArguments Args { get; }

        public static async Task Run(PatchTagCommandLineArguments args)
        {
            await new PatchTagTask(args).Run();
        }

        public PatchTagTask(PatchTagCommandLineArguments args)
        {
            this.Args = args;
        }

        public async Task Run()
        {

            using var inmemMap = new MemoryStream();
            using (var map = File.Open(this.Args.MapPath, FileMode.Open))
            {
                map.CopyTo(inmemMap);
                inmemMap.Position = 0;

                // Load to determine where to write patches to
                var factory = new MccMapFactory();
                this.scene = factory.FromStream(map);
            }

            var tagPatcher = new TagPatcher(scene, inmemMap);
            var settings = new JsonSerializerOptions() { ReadCommentHandling = JsonCommentHandling.Skip };
            var patches = JsonSerializer.Deserialize<TagPatch[]>(File.ReadAllText(this.Args.TagPatchPath), settings);
            foreach(var patch in patches)
                tagPatcher.Apply(patch);

            inmemMap.Position = 0;
            var sig = H2vMap.CalculateSignature(inmemMap.ToArray());
            inmemMap.WriteInt32At(BlamSerializer.StartsAt<H2vMapHeader>(h => h.StoredSignature), sig);
            inmemMap.Position = 0;

            using (var map = File.Open(this.Args.MapPath, FileMode.Open))
            {
                inmemMap.CopyTo(map);
            }
        }
    }
}
