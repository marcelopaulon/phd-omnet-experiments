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

network DADCA_ACK_n50_u4_a8000m2_3
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
        @display("i=misc/sensor;p=92.39,7853.46");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1215.11,7887.62");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1789.21,7768.57");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2402.95,7943.30");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4541.39,7960.55");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5678.79,7755.56");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=323.76,7579.33");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=762.11,7643.89");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1241.49,7511.76");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3182.35,7536.24");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3552.70,7591.72");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3971.39,7614.18");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4881.57,7536.83");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5193.99,7695.79");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1616.92,7365.67");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2261.67,7488.49");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2610.45,7283.68");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4391.28,7268.19");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=49.99,7083.34");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=496.35,7176.80");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=3100.32,7111.41");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3463.29,7126.06");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3830.47,7102.87");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5196.64,7101.06");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5680.61,7175.18");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=789.96,6805.51");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1550.39,6937.25");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2526.06,6791.26");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2893.09,6817.72");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=220.39,6556.53");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2154.57,6759.06");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=3362.96,6609.61");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=3739.18,6750.19");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=4143.48,6608.15");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4505.23,6718.01");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4826.73,6544.35");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=5410.39,6744.37");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5771.84,6755.38");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=650.07,6321.05");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1122.47,6439.70");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1543.75,6508.35");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2602.73,6413.83");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=5727.03,6365.04");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=84.51,6140.92");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1113.35,6027.86");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1569.23,6137.08");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=2049.80,6250.15");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=2771.79,6088.53");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3318.23,6258.03");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=3722.17,6071.50");
    }

}

