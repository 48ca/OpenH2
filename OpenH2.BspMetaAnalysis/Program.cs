﻿using OpenH2.Core.Factories;
using System.IO;
using System.Linq;
using System.Text;
using OpenH2.Translation;
using OpenH2.Translation.TagData;

namespace OpenH2.BspMetaAnalysis
{
    class Program
    {
        static void Main(string[] args)
        {
            var mapNames = new string[]
            {
                @"D:\Halo 2 Vista Original Maps\03a_oldmombasa.map"
            };

            var metas = mapNames.Select(s =>
            {
                var fac = new SceneFactory();
                using (var fs = new FileStream(s, FileMode.Open))
                    return fac.FromFile(fs);
            }).ToDictionary(s => s.Header.Name, s => s);

            var ascension = metas["03a_oldmombasa"];

            var translator = new TagTranslator(ascension);

            var bsps = translator.GetAll<BspTagData>().ToList();

            for(var i = 0; i < bsps.Count(); i++)
            {
                var bsp = bsps[i];

                var obj = CreatObjFileForBsp(bsp);
                File.WriteAllText($"D:\\bsp_{i}.obj", obj);
            }
        }

        public static string CreatObjFileForBsp(BspTagData mesh)
        {
            var sb = new StringBuilder();

            foreach (var vert in mesh.Verticies)
            {
                sb.AppendLine($"v {vert.Position.X.ToString("0.000000")} {vert.Position.Y.ToString("0.000000")} {vert.Position.Z.ToString("0.000000")}");
            }

            foreach (var tri in mesh.Faces)
            {
                sb.Append("f");

                foreach(var vert in tri)
                {
                    sb.Append($" {vert + 1}");
                }

                sb.AppendLine("");
            }

            return sb.ToString();
        }
    }
}
