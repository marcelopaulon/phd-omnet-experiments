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

network DADCA_ACK_n50_u3_a8000m2_7
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
        @display("i=misc/sensor;p=229.96,7858.03");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1181.45,7901.64");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1838.26,7942.76");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2839.13,7977.99");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4535.91,7867.91");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=944.95,7512.41");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1546.73,7661.50");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2169.11,7678.57");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2548.18,7735.52");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3416.68,7642.06");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3992.85,7562.65");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4890.91,7517.62");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5478.22,7737.26");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=268.84,7397.03");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2370.97,7376.74");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3121.36,7258.07");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4309.74,7403.79");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5806.38,7342.79");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=100.55,7023.35");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=603.93,7142.19");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1350.49,7131.71");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1993.17,7148.82");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=2660.16,7152.31");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3926.02,7180.44");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4669.47,7237.45");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=4951.45,7023.31");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=394.24,6831.15");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=1038.10,6928.87");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=1688.02,6838.22");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=2571.61,6794.07");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2995.52,6823.50");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=3380.19,6913.68");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4304.92,7008.76");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5430.51,7007.94");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5883.63,6824.96");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3509.88,6515.69");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=4071.34,6623.05");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4519.67,6555.06");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=5136.11,6605.99");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=542.29,6451.71");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1261.39,6439.05");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2099.73,6303.65");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=3006.90,6346.19");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=4771.63,6309.23");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=5661.88,6476.44");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=29.87,6109.49");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=852.41,6223.09");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4283.08,6128.04");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5416.88,6187.85");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=298.40,5778.82");
    }

}

