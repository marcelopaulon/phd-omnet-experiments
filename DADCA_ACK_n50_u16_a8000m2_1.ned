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

network DADCA_ACK_n50_u16_a8000m2_1
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
        @display("i=misc/sensor;p=410.69,7986.30");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=819.19,7841.58");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1557.80,7960.17");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2118.10,7799.96");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2570.78,7892.21");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3914.69,7989.04");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=100.63,7595.11");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1052.72,7516.07");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3078.79,7692.63");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3692.71,7676.39");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4654.50,7606.13");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5124.03,7750.02");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5666.91,7618.76");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=635.86,7369.84");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1973.65,7371.89");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4019.62,7471.33");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5226.50,7408.61");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1299.32,7240.79");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2498.52,7250.03");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3231.79,7221.63");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=3859.17,7044.55");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=4338.29,7069.13");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4978.50,7090.94");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5672.06,7089.36");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=75.41,6871.71");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=1040.26,7000.26");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1506.96,6802.85");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2092.96,6943.61");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3247.73,6792.23");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5318.48,6812.92");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1126.08,6595.82");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2251.93,6518.28");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2765.94,6724.68");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3585.87,6687.31");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4302.36,6678.79");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5699.25,6715.35");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=323.45,6322.11");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1447.44,6372.61");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1790.21,6474.82");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=5025.11,6475.99");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=5636.69,6276.21");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=897.74,6090.08");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1671.60,6074.86");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=2704.61,6087.99");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3132.77,6183.78");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3844.84,6222.36");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4391.42,6254.62");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=425.32,5842.22");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=2086.61,5989.04");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=3405.23,5831.05");
    }

}

