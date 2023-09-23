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

network DADCA_ACK_n50_u4_a8000m2_0
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
        @display("i=misc/sensor;p=989.01,7923.09");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2105.29,7991.45");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2439.71,7885.54");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4019.59,7793.82");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4365.19,7851.85");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5498.51,7824.57");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=189.54,7546.88");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=532.06,7686.72");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1460.53,7602.12");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2100.05,7560.31");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3671.75,7511.27");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4885.52,7626.60");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5266.25,7538.74");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5914.55,7748.11");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=445.66,7288.55");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=809.80,7445.33");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1751.94,7284.79");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2749.47,7409.76");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4053.33,7392.50");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4561.15,7280.86");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=5775.01,7369.57");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=106.79,7026.41");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=1180.10,7099.20");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=2122.76,7199.74");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=3441.02,7236.44");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5285.76,7045.90");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=672.87,6826.87");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=1854.69,6875.39");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2753.05,6990.51");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3912.67,6797.09");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=4366.65,6781.81");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=4704.51,6895.50");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=29.83,6562.86");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=1018.77,6705.28");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1445.77,6635.52");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2486.34,6585.43");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3120.72,6663.39");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3563.99,6519.46");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=5272.42,6687.16");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=5707.45,6737.45");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=500.45,6464.94");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=847.17,6338.12");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1581.72,6298.83");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=4007.03,6437.14");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4833.77,6342.94");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5183.10,6275.17");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=215.60,6149.16");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1928.68,6230.08");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3067.60,6240.79");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4345.51,6203.64");
    }

}

