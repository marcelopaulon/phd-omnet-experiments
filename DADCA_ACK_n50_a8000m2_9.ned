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

network DADCA_ACK_n50_a8000m2_9
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
        @display("i=misc/sensor;p=837.53,7972.50");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1760.72,7933.41");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3111.48,7769.63");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3673.53,7924.32");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4126.57,7762.27");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4506.65,7970.43");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4925.03,7768.66");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=308.31,7615.92");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1188.87,7734.84");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2279.83,7619.23");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5674.64,7717.27");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=903.70,7277.60");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1791.18,7392.25");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2910.21,7421.33");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3505.67,7353.66");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3968.92,7272.96");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4662.19,7337.76");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=87.61,7083.49");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1404.72,7101.54");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2316.73,7206.56");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2698.43,7064.62");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=4328.99,7129.89");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=5225.97,7227.66");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5563.15,7111.51");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5920.01,7167.21");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=388.75,6812.13");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=890.17,6831.62");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=1924.15,6964.84");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3030.67,6945.64");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3649.08,6995.71");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=4027.96,6841.50");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=4643.69,6955.11");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=5011.92,6916.67");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=1351.02,6678.68");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1670.39,6532.44");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2170.71,6620.54");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3725.83,6602.15");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4353.44,6700.70");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4929.72,6531.64");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=5619.93,6745.23");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=687.71,6345.53");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1130.52,6355.79");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2674.23,6465.66");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3343.89,6432.07");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1318.28,6032.88");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1917.80,6160.04");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=2309.23,6056.24");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=3018.05,6077.73");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3981.87,6116.21");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4546.92,6134.25");
    }

}

