using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MAVLinkAdapter.Core.Ardupilot
{
    public static class Extensions
    {
        public static T AwaitSync<T>(this Task<T> infunc)
        {
            var tsk = Task.Run<T>(async () =>
            {
                return await infunc.ConfigureAwait(false);
            });

            return tsk.GetAwaiter().GetResult();
        }
    }
}
