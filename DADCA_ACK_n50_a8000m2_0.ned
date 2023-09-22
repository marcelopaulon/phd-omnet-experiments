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

network DADCA_ACK_n50_a8000m2_0
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
        @display("i=misc/sensor;p=43.39,7898.79");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=664.07,7953.32");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1767.40,7947.89");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3408.96,7994.08");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3845.36,7875.00");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4816.32,7837.10");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5801.86,7813.91");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1233.53,7622.85");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2279.72,7618.38");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2649.93,7576.97");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3213.16,7574.85");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4523.23,7563.35");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5192.23,7547.68");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=292.05,7452.04");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=752.31,7414.07");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1097.90,7272.67");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1756.00,7296.92");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4188.73,7308.06");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5543.23,7366.11");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=359.13,6977.43");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2188.82,7090.41");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3166.23,6977.96");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=5030.77,7004.09");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5896.34,7045.35");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=1446.22,6785.94");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2560.31,6909.38");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3564.41,6939.20");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4110.70,6735.33");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4530.09,6838.51");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5416.38,6942.09");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=64.45,6645.49");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=756.74,6697.82");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1826.65,6633.83");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=2874.11,6686.07");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=3747.39,6539.22");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=490.04,6268.19");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=1278.43,6417.52");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1683.45,6251.94");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=2167.40,6446.47");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=3334.39,6403.88");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=4464.76,6433.12");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=4907.34,6320.11");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=5604.30,6246.71");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=1161.58,5989.40");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=2609.06,6025.93");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3314.19,6030.15");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3727.10,6011.17");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=5356.63,5971.77");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=878.32,5727.83");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=3081.09,5733.94");
    }

}

