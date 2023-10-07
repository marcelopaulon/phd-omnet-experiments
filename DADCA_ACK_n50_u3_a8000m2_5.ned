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

network DADCA_ACK_n50_u3_a8000m2_5
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
        @display("i=misc/sensor;p=2229.89,7889.65");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2834.86,7836.68");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=4002.91,7834.46");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=5568.43,7871.19");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=75.88,7608.48");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=661.65,7720.72");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1185.25,7566.60");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2045.09,7584.90");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3289.54,7712.35");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4329.36,7585.66");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5017.62,7678.76");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5953.75,7743.73");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=350.90,7379.28");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1654.34,7368.73");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3588.18,7269.65");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3968.87,7359.24");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5463.18,7399.78");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5901.28,7386.62");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=596.95,7074.82");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1036.06,7145.44");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2544.92,7230.88");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3034.66,7248.37");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4350.81,7118.27");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4782.57,7173.15");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=61.15,6999.81");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=1563.73,6875.34");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=2178.15,6967.61");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2846.59,6832.65");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3508.46,6897.26");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5118.94,6941.75");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5804.09,6880.28");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=343.78,6750.97");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=818.50,6612.80");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=2525.23,6652.48");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4076.97,6732.63");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4416.26,6578.71");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=4859.49,6578.03");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1187.48,6353.55");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1570.30,6272.29");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2997.12,6306.10");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3977.22,6287.39");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5266.14,6461.44");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=5955.85,6465.37");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=100.80,6266.08");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=663.24,6144.88");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2003.03,6076.90");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=2461.38,6103.49");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=3629.51,6113.03");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=4379.82,6090.09");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5625.34,6192.55");
    }

}

