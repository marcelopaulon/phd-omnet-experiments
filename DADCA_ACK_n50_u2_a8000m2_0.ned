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

network DADCA_ACK_n50_u2_a8000m2_0
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
        @display("i=misc/sensor;p=50.02,7917.49");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=423.77,7870.43");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1270.45,7788.41");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2403.45,7895.86");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3578.84,7833.63");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3949.70,7886.86");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4536.01,7777.74");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5718.89,7836.41");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=64.48,7507.68");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=855.29,7645.74");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1837.17,7731.50");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3150.65,7676.57");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=483.54,7454.01");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2139.97,7366.03");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3693.66,7364.98");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4303.26,7316.03");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5136.63,7478.16");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5665.17,7258.66");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=853.89,7238.48");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1499.25,7211.33");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2565.68,7217.69");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3335.03,7150.56");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4595.00,7040.75");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5010.58,7056.00");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=246.16,6851.54");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2012.06,6823.88");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=2381.06,6845.88");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2740.29,6908.78");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3876.96,6911.83");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4271.43,6886.58");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5862.07,6889.76");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=624.00,6582.81");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1072.95,6679.25");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3035.67,6646.33");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=3481.95,6539.41");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4821.97,6669.37");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=5445.83,6537.89");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=151.04,6496.31");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=2015.46,6425.27");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2659.89,6460.33");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3986.37,6391.09");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=4346.83,6463.15");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=127.49,6057.37");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=536.63,6196.61");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1069.04,6207.12");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2958.54,6239.02");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3784.60,6093.11");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=5230.03,6220.14");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1309.28,5840.55");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1636.91,5963.94");
    }

}

