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

network DADCA_ACK_n50_u3_a8000m2_9
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
        @display("i=misc/sensor;p=683.73,7866.28");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1246.64,7919.05");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1835.94,7821.98");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2907.81,7802.19");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3352.48,7844.01");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4341.42,7797.32");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=45.07,7615.02");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=3820.42,7687.21");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4900.81,7582.57");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=363.16,7453.24");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1014.53,7350.13");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1470.80,7372.26");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3314.35,7463.60");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3653.38,7323.74");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4591.24,7415.39");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5381.27,7452.09");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5847.76,7404.46");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1991.34,7204.53");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2902.26,7013.20");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3426.67,7040.77");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=4396.38,7090.42");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=4761.49,7038.92");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=5473.43,7087.19");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=962.49,6980.43");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=1426.40,6820.48");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2078.53,6775.39");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3824.33,6913.01");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=207.40,6694.68");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=1103.34,6560.48");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=1728.60,6525.44");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2599.38,6756.79");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=3149.92,6682.72");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4129.48,6660.55");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5321.64,6743.03");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5807.85,6522.98");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=735.37,6432.58");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2617.57,6315.32");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3466.62,6416.42");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4697.60,6449.76");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=373.47,6188.26");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1060.38,6166.54");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1700.57,6151.32");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2047.89,6093.99");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=2933.45,6116.69");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3296.58,6069.84");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3889.01,6153.44");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=979.98,5814.55");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=2419.61,5889.49");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=4240.89,6011.60");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4804.20,5894.06");
    }

}

