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
        @display("i=misc/sensor;p=112.77,7914.21");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=750.24,7845.63");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1774.73,7852.10");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2283.82,7986.97");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=877.87,7514.11");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=2315.98,7585.43");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=2799.47,7738.98");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=3464.45,7603.10");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3998.42,7746.99");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4708.07,7680.18");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5321.63,7675.69");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1288.02,7454.62");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1963.90,7327.29");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3162.79,7380.79");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3845.96,7369.85");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4319.84,7434.02");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5038.99,7447.76");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5360.08,7269.90");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5645.84,7488.45");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=806.34,7105.07");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2430.22,7248.22");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=4668.85,7203.95");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=250.41,6969.79");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=1342.16,6970.28");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=1778.76,6962.59");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2300.29,6856.18");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3005.14,6957.52");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3686.39,6970.62");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4062.03,6940.34");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4445.23,6922.53");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5013.81,6938.20");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=5905.33,6954.05");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2664.94,6717.96");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3306.71,6539.46");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=3964.25,6531.34");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4467.50,6561.53");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=4923.94,6534.99");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5281.84,6670.47");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=5640.43,6608.90");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=44.74,6444.55");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1083.78,6485.86");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1438.60,6507.62");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1809.35,6390.23");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=2434.20,6453.36");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3001.99,6269.77");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3630.53,6368.38");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=35.22,6050.78");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=682.80,6140.46");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1274.87,6130.37");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1901.73,6023.96");
    }

}

