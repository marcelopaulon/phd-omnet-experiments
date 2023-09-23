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

network DADCA_ACK_n50_u2_a8000m2_5
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
        @display("i=misc/sensor;p=499.92,7893.90");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1244.39,7988.99");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1631.19,7785.49");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2406.32,7975.81");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2765.94,7794.11");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3297.37,7776.47");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=3976.80,7869.90");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5984.31,7873.91");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=172.55,7516.20");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1074.67,7588.40");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2268.98,7610.31");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3697.31,7645.70");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4279.83,7641.15");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4776.24,7656.52");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=5402.46,7628.54");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=753.86,7285.07");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1884.52,7346.36");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2606.33,7367.02");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3618.78,7276.27");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5221.96,7275.70");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=5838.54,7470.70");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=327.17,7138.82");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=1291.68,7168.97");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=1681.31,7019.35");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=3226.65,7147.15");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=3981.73,7017.81");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=4343.43,7184.47");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4706.67,7052.85");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=84.41,6844.71");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=813.39,6909.84");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2134.65,6970.50");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2661.41,6883.83");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=5609.34,6945.93");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3461.50,6550.51");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=3832.72,6538.92");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5081.47,6733.00");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=5396.43,6528.00");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5806.42,6610.30");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=306.95,6425.84");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1224.25,6458.01");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2273.40,6428.80");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2780.05,6290.74");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4403.90,6505.92");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=4778.59,6351.18");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=189.19,6065.83");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=876.91,6196.64");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=1475.95,6167.96");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=3400.18,6076.07");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3965.41,6179.45");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4393.78,6086.46");
    }

}

