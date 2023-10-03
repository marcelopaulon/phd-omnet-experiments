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

network DADCA_ACK_n50_u3_a8000m2_2
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
        @display("i=misc/sensor;p=1189.79,7785.77");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1856.15,7794.58");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=4443.31,7818.43");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2522.62,7623.40");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2959.59,7717.68");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5336.02,7639.09");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1570.91,7453.58");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1948.68,7341.96");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3067.05,7295.64");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3674.68,7362.35");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4153.45,7339.83");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4653.23,7453.29");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5222.46,7279.04");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=488.26,7217.32");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=961.33,7232.81");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1378.56,7121.23");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3340.07,7063.42");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3900.76,7038.02");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5742.03,7204.28");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=669.16,6787.94");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1865.03,6870.05");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2367.36,6980.19");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=2937.98,6884.81");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4513.73,6810.94");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5075.03,6840.69");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=1142.43,6757.95");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1540.31,6703.91");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3377.08,6543.91");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5336.89,6541.68");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5710.42,6729.35");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=224.17,6388.00");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=705.15,6386.10");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1297.55,6284.98");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=1813.68,6445.08");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=2180.15,6394.37");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2851.41,6491.70");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3881.10,6424.41");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4937.01,6383.47");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=2331.81,6044.61");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2682.93,6147.88");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3212.39,6197.20");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=4022.46,6028.22");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=5102.49,6065.00");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5764.28,6203.14");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=467.17,5819.34");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1748.60,5913.08");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3171.38,5826.18");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=3543.74,5832.56");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=4438.38,6020.01");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5431.19,5923.98");
    }

}

