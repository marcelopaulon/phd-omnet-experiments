package projeto;

import inet.common.geometry.common.SimpleGeographicCoordinateSystem;
import inet.environment.common.PhysicalEnvironment;
import inet.environment.ground.FlatGround;
import inet.node.inet.AdhocHost;
import inet.visualizer.integrated.IntegratedVisualizer;
import inet.mobility.contract.IMobility;
import inet.mobility.static.StationaryMobility;
import inet.networklayer.configurator.ipv4.Ipv4NetworkConfigurator;
import inet.node.ethernet.Eth100M;
import inet.node.inet.SensorNode;
import inet.node.inet.StandardHost;
import inet.node.inet.WirelessHost;
import inet.physicallayer.apskradio.packetlevel.ApskScalarRadioMedium;
import inet.physicallayer.ieee802154.bitlevel.Ieee802154UwbIrRadioMedium;
import inet.physicallayer.ieee802154.packetlevel.Ieee802154NarrowbandScalarRadioMedium;
import inet.physicallayer.ieee80211.packetlevel.Ieee80211ScalarRadioMedium;
import inet.visualizer.integrated.IntegratedMultiVisualizer;
import inet.visualizer.integrated.IntegratedVisualizer;
import inet.physicallayer.contract.packetlevel.IRadioMedium;

network DADCA_ACK_n50_u8_a8000m2_3
{
    parameters:
		int numUAVs = default(1);
    submodules:
        visualizer: IntegratedVisualizer {
            parameters:
                @display("p=100,50");
        }
        coordinateSystem: SimpleGeographicCoordinateSystem {
            parameters:
                @display("p=100,150");
        }
        
        environment: PhysicalEnvironment {
        }
        
        configurator: Ipv4NetworkConfigurator {
            @display("p=0,200");
        }
        radioMedium: <default("Ieee802154NarrowbandScalarRadioMedium")> like IRadioMedium {
            @display("p=200,0");
        }
        quads[numUAVs]: MobileNode {
			
        } 
        
        groundStation: GroundStation {
            @display("i=misc/sensor3");
        }
       
        
    sensors_1: MobileSensorNode {
        @display("i=misc/sensor;p=1592.79,7843.79");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2943.24,7991.15");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3469.64,7915.68");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4572.89,7936.71");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5115.43,7978.82");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=177.25,7597.18");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=661.99,7543.51");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2200.35,7623.87");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2848.78,7521.15");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4119.02,7694.77");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4851.02,7650.71");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5439.61,7602.68");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5860.24,7616.36");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1264.60,7444.05");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1749.10,7263.70");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3431.20,7398.29");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3909.30,7382.87");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4579.38,7304.91");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=237.88,7134.15");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1426.46,7033.40");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2494.55,7250.84");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3309.13,7054.93");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4133.23,7040.58");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4914.56,7184.70");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5583.46,7240.20");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=822.64,6905.47");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=2345.49,6909.48");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2791.94,6777.17");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5130.97,6885.41");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=203.90,6703.74");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1646.01,6625.84");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=3771.57,6580.73");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4632.08,6750.18");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=468.19,6324.27");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1267.40,6437.21");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2460.72,6311.37");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3107.44,6457.52");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4371.51,6390.28");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=5006.67,6504.42");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1565.04,6065.53");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2044.27,6049.29");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3053.13,6056.30");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=3533.24,6264.41");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=4153.49,6063.70");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=5732.84,6251.92");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=294.32,5811.12");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=944.90,5838.68");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=3384.26,5880.01");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3771.40,5905.77");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4772.59,5927.61");
    }

}

