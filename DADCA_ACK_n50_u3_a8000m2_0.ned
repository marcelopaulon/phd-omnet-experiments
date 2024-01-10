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

network DADCA_ACK_n50_u3_a8000m2_0
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
        @display("i=misc/sensor;p=380.21,7958.75");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1453.66,7992.25");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1753.46,7766.86");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4099.74,7777.39");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5183.02,7931.10");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5928.61,7818.39");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=76.09,7657.48");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=616.47,7557.31");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1157.54,7744.69");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2119.74,7725.66");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2827.27,7724.26");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3684.43,7566.19");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4722.22,7511.81");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5327.74,7507.69");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1199.40,7345.42");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2407.21,7312.21");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2937.17,7361.32");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5792.16,7385.85");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=82.76,7151.44");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=789.94,7165.32");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1865.24,7198.15");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3312.61,7252.65");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4066.98,7114.51");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5004.79,7129.43");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5412.88,7152.25");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=1444.11,6837.53");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=2427.71,6847.93");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3740.83,6789.05");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4416.74,6894.42");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4838.73,6808.41");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5886.14,6900.69");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=354.23,6665.51");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1100.02,6564.76");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=2912.59,6661.46");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=3312.13,6722.23");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4265.73,6537.23");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=5223.04,6673.34");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5615.53,6553.75");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=77.00,6299.27");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1588.40,6422.80");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1928.74,6508.52");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2291.12,6307.72");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2659.96,6340.80");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3664.16,6385.68");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4748.71,6459.69");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=529.81,6206.02");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=1183.31,6074.34");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=3319.90,6236.48");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=4522.56,6141.11");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4891.35,6139.18");
    }

}

