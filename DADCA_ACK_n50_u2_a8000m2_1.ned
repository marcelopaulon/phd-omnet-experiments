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

network DADCA_ACK_n50_u2_a8000m2_1
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
        @display("i=misc/sensor;p=1223.80,7898.80");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1652.28,7988.05");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3103.18,7957.59");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3478.68,7969.18");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4117.20,7901.43");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5116.34,7958.29");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=554.54,7600.56");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=970.12,7560.37");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1468.96,7629.92");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2195.54,7594.81");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2611.71,7649.71");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3364.46,7571.62");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5473.63,7648.68");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=110.53,7343.81");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1834.59,7430.78");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3015.93,7387.06");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4155.39,7452.24");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4925.89,7496.14");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5254.77,7330.37");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1160.08,7159.07");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1512.93,7162.50");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1937.87,7050.93");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=2328.06,7157.88");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3791.19,7161.18");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4167.96,7088.65");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=4771.30,7180.92");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=5761.78,7160.02");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=733.46,6958.00");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3395.17,7004.46");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5057.63,6859.87");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5408.01,6972.76");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=255.33,6698.13");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=926.17,6553.06");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=1333.93,6555.63");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1916.67,6645.05");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2572.20,6580.10");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3067.24,6697.07");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4189.07,6656.96");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4691.08,6602.32");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=5584.74,6628.04");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=280.77,6289.19");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2236.52,6291.79");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2965.31,6303.31");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3608.47,6463.93");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=5028.74,6307.96");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5917.41,6349.23");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=759.56,6206.79");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1425.70,6156.30");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1776.41,6042.38");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=2636.11,6031.29");
    }

}

