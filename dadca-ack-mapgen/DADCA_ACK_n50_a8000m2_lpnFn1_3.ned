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

network DADCA_ACK_n50_a8000m2_lpnFn1_3
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
        @display("i=misc/sensor;p=100.00,100.00");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=86.64,36.86");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=7.46,77.12");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=45.94,129.25");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=64.76,211.93");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3.00,220.88");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=135.02,309.28");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=120.80,250.13");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=135.59,170.77");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=189.73,124.35");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=252.22,186.15");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=225.31,269.59");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=288.23,244.53");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=356.50,313.50");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=369.97,221.78");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=336.83,155.52");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=265.67,122.77");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=34.95,361.44");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=20.95,428.84");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=91.65,498.97");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=42.93,550.53");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=121.99,396.44");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=202.01,387.16");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=263.54,370.05");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=423.33,149.41");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=499.39,211.62");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=487.93,118.95");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=486.60,32.55");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=572.48,83.51");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=692.64,76.23");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=765.71,50.72");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=852.98,88.62");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=687.10,139.96");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=577.05,179.24");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=566.54,324.84");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=499.37,317.62");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=665.47,375.25");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=553.39,255.62");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=635.20,300.90");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=701.22,448.01");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=610.70,420.85");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=705.98,329.59");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=787.16,457.97");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=733.65,499.40");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=756.77,365.10");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=732.40,208.59");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=686.07,264.62");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=835.68,203.49");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=833.90,281.78");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=765.70,282.19");
    }

}

