using Jitter.LinearMath;
using System.Collections.Generic;
using System.Diagnostics;

namespace Jitter.Dynamics
{
    public class ContactList : List<Contact>
    {
        public ContactList() : base(4) { }
    }

    public class Arbiter
    {
        public RigidBody Body1 => body1;

        public RigidBody Body2 => body2;

        public ContactList ContactList => contactList;

        public static ResourcePool<Arbiter> Pool = new ResourcePool<Arbiter>();

        internal RigidBody body1;
        internal RigidBody body2;
        internal ContactList contactList;

        public Arbiter(RigidBody body1, RigidBody body2)
        {
            contactList = new ContactList();
            this.body1 = body1;
            this.body2 = body2;
        }

        public Arbiter()
        {
            contactList = new ContactList();
        }

        public void Invalidate()
        {
            contactList.Clear();
        }

        public Contact AddContact(
            JVector point1,
            JVector point2,
            JVector normal,
            float penetration,
            ContactSettings contactSettings)
        {
            JVector.Subtract(point1, body1.position, out var relPos1);

            int index;

            lock (contactList)
            {
                if (contactList.Count == 4)
                {
                    index = SortCachedPoints(relPos1, penetration);
                    ReplaceContact(point1, point2, normal, penetration, index, contactSettings);
                    return null;
                }

                index = GetCacheEntry(relPos1, contactSettings.breakThreshold);

                if (index >= 0)
                {
                    ReplaceContact(point1, point2, normal, penetration, index, contactSettings);
                    return null;
                }
                else
                {
                    var contact = Contact.Pool.GetNew();
                    contact.Initialize(body1, body2, point1, point2, normal, penetration, true, contactSettings);
                    contactList.Add(contact);
                    return contact;
                }
            }
        }

        private void ReplaceContact(
            in JVector point1,
            in JVector point2,
            in JVector n,
            float p,
            int index,
            ContactSettings contactSettings)
        {
            var contact = contactList[index];

            Debug.Assert(body1 == contact.body1, "Body1 and Body2 not consistent.");

            contact.Initialize(body1, body2, point1, point2, n, p, false, contactSettings);
        }

        private int GetCacheEntry(in JVector realRelPos1, float contactBreakThreshold)
        {
            float shortestDist = contactBreakThreshold * contactBreakThreshold;
            int size = contactList.Count;
            int nearestPoint = -1;
            for (int i = 0; i < size; i++)
            {
                JVector.Subtract(contactList[i].relativePos1, realRelPos1, out var diffA);
                float distToManiPoint = diffA.LengthSquared();
                if (distToManiPoint < shortestDist)
                {
                    shortestDist = distToManiPoint;
                    nearestPoint = i;
                }
            }
            return nearestPoint;
        }

        private int SortCachedPoints(in JVector realRelPos1, float pen)
        {
            int maxPenetrationIndex = -1;
            float maxPenetration = pen;
            for (int i = 0; i < 4; i++)
            {
                if (contactList[i].penetration > maxPenetration)
                {
                    maxPenetrationIndex = i;
                    maxPenetration = contactList[i].penetration;
                }
            }

            float res0 = 0;
            float res1 = 0;
            float res2 = 0;
            float res3 = 0;
            if (maxPenetrationIndex != 0)
            {
                JVector.Subtract(realRelPos1, contactList[1].relativePos1, out var a0);
                JVector.Subtract(contactList[3].relativePos1, contactList[2].relativePos1, out var b0);
                JVector.Cross(a0, b0, out var cross);
                res0 = cross.LengthSquared();
            }
            if (maxPenetrationIndex != 1)
            {
                JVector.Subtract(realRelPos1, contactList[0].relativePos1, out var a0);
                JVector.Subtract(contactList[3].relativePos1, contactList[2].relativePos1, out var b0);
                JVector.Cross(a0, b0, out var cross);
                res1 = cross.LengthSquared();
            }

            if (maxPenetrationIndex != 2)
            {
                JVector.Subtract(realRelPos1, contactList[0].relativePos1, out var a0);
                JVector.Subtract(contactList[3].relativePos1, contactList[1].relativePos1, out var b0);
                JVector.Cross(a0, b0, out var cross);
                res2 = cross.LengthSquared();
            }

            if (maxPenetrationIndex != 3)
            {
                JVector.Subtract(realRelPos1, contactList[0].relativePos1, out var a0);
                JVector.Subtract(contactList[2].relativePos1, contactList[1].relativePos1, out var b0);
                JVector.Cross(a0, b0, out var cross);
                res3 = cross.LengthSquared();
            }

            int biggestarea = MaxAxis(res0, res1, res2, res3);
            return biggestarea;
        }

        internal static int MaxAxis(float x, float y, float z, float w)
        {
            int maxIndex = -1;
            float maxVal = float.MinValue;

            if (x > maxVal)
            {
                maxIndex = 0;
                maxVal = x;
            }
            if (y > maxVal)
            {
                maxIndex = 1;
                maxVal = y;
            }
            if (z > maxVal)
            {
                maxIndex = 2;
                maxVal = z;
            }
            if (w > maxVal)
            {
                maxIndex = 3;
            }

            return maxIndex;
        }
    }
}
