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

network DADCA_ACK_n50_u4_a8000m2_8
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
        @display("i=misc/sensor;p=393.05,7808.37");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1183.14,7966.54");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1498.33,7806.04");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3072.98,7892.97");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4482.60,7764.80");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5072.30,7793.64");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=2467.93,7679.54");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=3685.82,7730.23");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4063.78,7534.16");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5703.92,7664.49");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=194.71,7497.22");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=621.63,7407.38");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1118.16,7430.66");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1941.43,7489.89");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2920.62,7270.56");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3284.18,7272.03");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4756.24,7367.69");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5353.34,7491.33");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=736.41,7070.96");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2443.47,7187.86");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=3762.43,7112.95");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=4119.49,7047.58");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=5509.45,7145.15");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=377.59,6763.58");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=1523.83,6957.66");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=1989.33,6947.42");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=2564.58,6783.16");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4754.94,6943.07");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5147.55,6929.57");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5841.62,6997.15");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=766.25,6550.61");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1323.47,6537.77");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2198.38,6605.02");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3322.05,6758.02");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5536.93,6612.14");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1717.54,6461.49");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2732.87,6331.71");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4168.23,6453.74");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4520.18,6281.46");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4886.76,6508.64");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=5207.11,6328.33");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5917.34,6505.10");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=423.10,6051.82");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=889.41,6134.33");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1243.08,6106.79");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2169.83,6115.71");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3650.18,6220.77");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4815.14,6032.49");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1773.00,5906.18");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=2744.88,5939.33");
    }

}

