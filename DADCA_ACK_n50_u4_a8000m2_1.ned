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

network DADCA_ACK_n50_u4_a8000m2_1
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
        @display("i=misc/sensor;p=753.29,7938.36");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1249.05,7948.83");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1630.09,7791.16");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2129.49,7887.07");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4651.39,7798.40");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=2887.83,7509.46");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=3283.66,7606.00");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=3932.34,7747.61");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5048.65,7565.52");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=330.57,7440.11");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=981.31,7487.21");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1412.41,7443.22");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2207.46,7473.02");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3814.75,7378.18");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4402.26,7399.04");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5756.34,7270.50");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1262.38,7106.73");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1727.54,7234.54");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2170.70,7034.50");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2478.98,7221.37");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=3501.90,7167.60");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=4128.67,7038.74");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4753.61,7219.73");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=51.87,6948.99");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=720.10,6946.46");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=1600.64,6800.45");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3163.85,6859.39");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3736.87,6879.90");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4626.24,6843.02");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5057.48,6874.89");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5434.92,7001.88");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=5920.09,6822.51");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=185.29,6565.12");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=1239.42,6666.14");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=2260.81,6665.73");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2679.99,6733.54");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=4077.59,6688.03");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5642.41,6568.82");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=582.14,6291.66");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1765.63,6310.99");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2879.87,6411.34");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3231.96,6397.37");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=3660.81,6510.07");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5217.24,6412.57");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=5945.65,6363.39");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2337.30,6217.25");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3344.76,6029.61");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=3762.13,6152.58");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=4127.99,6235.80");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5371.82,6096.93");
    }

}

