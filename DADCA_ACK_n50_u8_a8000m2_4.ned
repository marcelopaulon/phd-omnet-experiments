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

network DADCA_ACK_n50_u8_a8000m2_4
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
        @display("i=misc/sensor;p=99.42,7786.15");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=4131.15,7765.85");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=5087.99,7795.18");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=1223.90,7688.23");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2130.77,7639.83");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=2569.36,7693.60");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4405.93,7534.97");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=640.45,7365.75");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1762.17,7393.03");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2382.02,7295.30");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2958.69,7422.34");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3989.52,7305.89");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4853.06,7481.46");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5408.21,7387.14");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=304.85,7168.53");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1122.99,7256.63");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1585.84,7024.76");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2118.57,7024.88");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2742.15,7088.00");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3571.16,7061.18");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=548.99,6810.50");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1025.86,6885.36");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=2446.02,6888.11");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3134.53,6947.87");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4078.76,6780.64");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=4553.88,6989.03");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=5210.70,6959.96");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=52.90,6549.40");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=1674.51,6532.81");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=2123.87,6668.77");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2814.18,6616.48");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=3492.00,6559.83");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=5570.65,6736.27");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=480.06,6287.69");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1331.85,6405.58");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4396.50,6400.89");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=5337.36,6473.36");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=11.47,6039.50");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=926.36,6211.59");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1915.24,6130.58");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3088.20,6084.07");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=4063.47,6171.72");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4784.62,6215.05");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5523.92,6125.59");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=5892.25,6120.90");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1514.87,5989.35");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=2339.34,5868.38");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=3715.23,5892.39");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=4731.88,5815.96");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5119.00,5942.02");
    }

}

