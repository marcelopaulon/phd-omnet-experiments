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

network DADCA_ACK_n50_u8_a8000m2_7
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
        @display("i=misc/sensor;p=58.59,7827.88");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=491.08,7864.36");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=878.05,7998.59");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=1561.86,7994.39");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2837.56,7829.63");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3415.48,7905.13");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=3961.20,7946.55");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4297.99,7825.08");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=707.21,7570.45");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1169.11,7586.53");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1805.68,7705.82");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4703.23,7743.41");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5560.93,7677.93");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=320.69,7430.76");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1571.66,7364.50");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2113.77,7475.49");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2846.88,7265.84");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3200.14,7409.05");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3556.04,7454.68");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4124.01,7496.87");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=5124.57,7389.20");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=13.13,7199.15");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=509.21,7113.44");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=1924.26,7041.88");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2585.55,7022.07");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=4545.13,7171.83");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=5440.15,7204.85");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=5.19,6818.95");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=822.87,6917.05");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=1402.00,6877.77");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=3351.37,6908.41");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=3789.52,6899.88");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4257.72,6887.03");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5057.31,6964.38");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5993.37,6863.14");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1954.21,6660.65");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2388.59,6586.91");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=2951.62,6518.92");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4725.88,6672.53");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=5292.00,6558.07");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=5693.84,6605.57");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=66.85,6349.28");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=447.75,6457.21");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=1362.48,6467.54");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3620.04,6402.84");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=4154.48,6418.08");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=330.15,6118.63");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=806.50,6190.92");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1499.61,6118.12");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1967.64,6176.26");
    }

}

