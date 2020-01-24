﻿using System.Collections.Generic;
using System.Numerics;
using OpenH2.Core.Tags.Scenario;
using OpenH2.Foundation;

namespace OpenH2.Rendering.Abstractions
{
    /// <summary>
    /// Accumulates resources for each frame and orders/groups/batches calls to the <see cref="IGraphicsAdapter"/>
    /// </summary>
    public interface IRenderingPipeline<TMaterialMap>
    {
        void SetModels(IList<(Model<TMaterialMap>, Matrix4x4)> models);
        void AddPointLight(PointLight light);

        //void AddUI();

        void DrawAndFlush();
    }
}
