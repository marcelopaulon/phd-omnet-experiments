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

network DADCA_ACK_n50_u4_a8000m2_4
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
        @display("i=misc/sensor;p=76.46,7908.09");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=457.84,7809.94");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1878.01,7822.88");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2246.83,7940.18");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3103.11,7867.55");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3618.31,7766.22");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4123.24,7881.80");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=979.55,7685.66");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1539.67,7685.85");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2986.99,7532.31");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4549.07,7681.29");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=392.32,7441.59");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=781.50,7347.07");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1896.08,7333.85");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2324.85,7455.33");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3954.60,7398.14");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=68.65,7187.95");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1226.22,7182.01");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2578.34,7194.40");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3483.88,7092.13");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=4604.75,7089.00");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=4959.22,7091.93");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=1168.55,6805.20");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=1878.13,6940.97");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2294.27,6943.01");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=3253.20,6767.80");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=4076.92,6766.17");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=5863.88,6947.65");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=169.68,6538.72");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=502.13,6735.53");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1768.86,6535.12");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2271.77,6519.06");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2652.14,6755.02");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3711.85,6556.38");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4513.41,6682.59");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4918.81,6715.20");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=5390.40,6754.47");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=2635.84,6375.04");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=3186.83,6372.46");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4408.82,6277.13");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=5024.10,6316.24");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=430.23,6214.39");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1012.50,6261.05");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3790.79,6138.45");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=5702.96,6182.09");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1248.00,5859.08");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=1882.93,5867.73");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=2522.67,5887.63");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=4038.91,5854.24");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4430.45,5772.64");
    }

}

