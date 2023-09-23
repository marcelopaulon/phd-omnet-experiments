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

network DADCA_ACK_n50_u2_a8000m2_8
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
        @display("i=misc/sensor;p=99.97,7963.09");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=730.48,7763.71");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1268.90,7843.46");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=1825.15,7867.98");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2911.61,7866.76");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3310.52,7821.49");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5105.45,7949.35");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=333.70,7516.35");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1031.43,7534.41");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2445.73,7614.60");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2814.43,7528.80");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3787.74,7644.17");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5339.68,7525.53");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5898.17,7615.91");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=685.84,7412.24");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1551.41,7341.68");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2049.60,7288.89");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3212.13,7296.95");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4971.53,7279.92");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=982.30,7168.89");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2337.05,7069.77");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2808.42,7131.39");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4272.97,7198.02");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5675.66,7072.66");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=695.74,6829.10");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=1566.72,6928.08");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3641.49,6974.89");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4056.16,6904.42");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4448.83,6829.90");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=256.68,6753.58");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2120.79,6720.57");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2647.16,6594.58");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=3191.52,6709.69");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3754.32,6535.83");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4833.82,6667.84");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5194.68,6665.89");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=731.63,6468.91");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1377.01,6514.54");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1789.29,6271.51");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=3316.02,6370.91");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=4204.75,6458.41");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=4665.91,6343.83");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=5124.03,6270.45");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=170.00,6164.47");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=543.28,6156.54");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1020.71,6251.46");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=2172.25,6114.44");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=2673.19,6145.50");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5568.50,6211.79");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=174.64,5791.74");
    }

}

