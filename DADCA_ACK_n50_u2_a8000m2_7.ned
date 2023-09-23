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

network DADCA_ACK_n50_u2_a8000m2_7
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
        @display("i=misc/sensor;p=46.88,7820.72");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=949.35,7858.09");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1343.41,7859.19");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=1866.86,7885.50");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2724.86,7882.12");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3139.85,7899.68");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=3886.76,7874.63");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4410.59,7925.02");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4819.32,7894.53");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5165.24,7801.40");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5710.19,7876.11");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=405.67,7690.88");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1783.51,7532.92");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2291.58,7699.09");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2937.26,7562.12");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3493.04,7577.97");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=82.37,7349.00");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=814.95,7465.07");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1205.48,7266.79");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2610.59,7307.10");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=4071.15,7499.48");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=4510.12,7438.94");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4962.53,7348.90");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5696.26,7373.98");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=616.39,7062.76");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2088.69,7245.57");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=2988.31,7042.36");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3384.18,7210.26");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3783.38,7241.35");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=1039.74,6921.77");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1630.58,6864.66");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2325.46,6906.70");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4169.94,6780.99");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=4729.85,6874.10");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5306.00,6912.00");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5839.94,6872.35");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=442.31,6749.19");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1013.19,6557.93");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=2714.75,6543.89");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=3606.46,6663.85");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=4612.61,6531.69");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=193.37,6465.66");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1356.12,6381.09");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=1769.46,6340.65");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=2433.42,6316.42");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=4071.62,6289.92");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=5520.15,6383.98");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=216.11,6092.95");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=610.19,6234.02");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=2159.97,6070.34");
    }

}

