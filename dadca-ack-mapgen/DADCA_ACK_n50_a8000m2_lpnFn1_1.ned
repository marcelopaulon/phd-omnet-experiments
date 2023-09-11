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

network DADCA_ACK_n50_a8000m2_lpnFn1_1
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
        @display("i=misc/sensor,p=100.00,100.00");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor,p=77.27,161.83");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=163.43,137.14");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=139.12,243.92");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=198.98,208.78");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=259.35,212.11");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=295.49,267.58");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=194.90,335.99");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=113.18,306.01");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=182.21,432.55");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=237.04,380.09");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=266.05,447.44");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=217.82,499.43");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=72.70,498.68");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=0.69,534.04");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=59.95,581.40");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=46.46,675.47");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=9.43,761.15");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=58.91,805.01");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=54.20,884.74");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=84.82,735.49");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=187.38,773.45");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=274.12,810.83");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=344.49,754.75");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=412.94,717.73");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=441.05,621.94");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=481.91,572.47");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=488.69,496.73");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=404.17,516.57");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=369.39,582.39");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=267.26,616.36");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=205.51,663.86");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=150.06,639.68");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=298.98,710.66");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=419.03,834.64");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=500.37,832.51");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=513.23,918.03");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=436.09,943.55");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=541.14,986.27");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=561.31,1094.66");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=641.41,1010.61");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=626.77,934.36");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=613.76,856.95");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=492.17,1119.16");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=432.62,1093.56");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=579.63,1200.78");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=644.68,1222.36");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=578.63,1282.80");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=432.29,1187.54");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=648.46,1153.63");
    }

}

