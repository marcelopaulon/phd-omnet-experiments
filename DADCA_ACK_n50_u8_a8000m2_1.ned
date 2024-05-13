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

network DADCA_ACK_n50_u8_a8000m2_1
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
        @display("i=misc/sensor;p=421.96,7808.58");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1198.44,7907.75");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1557.12,7800.21");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2850.02,7809.61");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3488.06,7772.30");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4842.51,7984.86");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5262.77,7981.43");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=902.55,7634.82");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2505.11,7642.94");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4150.65,7748.36");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4651.85,7603.88");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5119.12,7640.36");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5582.02,7571.90");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5935.74,7646.17");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=472.36,7374.95");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1732.32,7484.74");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2660.63,7261.57");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3153.75,7306.68");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3674.70,7333.43");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4102.96,7332.62");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1206.44,7172.92");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2276.28,7119.24");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=2929.47,7018.44");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4502.74,7166.59");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4883.95,7191.14");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5883.44,7192.65");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=666.40,7009.93");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=983.76,6789.46");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=1556.01,6903.40");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=1895.39,6805.72");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=3484.15,6831.48");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=3855.35,6800.88");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=5331.79,6960.22");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5985.62,6810.09");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=132.69,6671.44");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1350.35,6527.42");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2510.96,6547.22");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3128.19,6552.92");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4263.36,6559.91");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4801.68,6541.82");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=313.59,6286.07");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=710.05,6410.48");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2086.35,6474.37");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3638.50,6508.03");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=5535.93,6278.07");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1061.38,6214.41");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=1776.13,6139.34");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=2823.17,6209.93");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3336.19,6167.59");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=3908.56,6242.49");
    }

}

