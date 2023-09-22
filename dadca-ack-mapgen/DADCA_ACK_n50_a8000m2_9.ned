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
        @display("i=misc/sensor;p=100.85,7962.97");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=190.55,7996.50");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=383.26,7980.51");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=539.08,7972.18");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=612.69,7965.21");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=728.92,7960.37");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=860.39,7964.26");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=164.91,7937.87");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=252.01,7954.10");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=432.53,7930.11");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=798.91,7953.78");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=902.08,7918.96");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=954.92,7954.07");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=155.03,7874.91");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=236.88,7883.49");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=332.68,7906.99");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=547.53,7885.49");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=705.68,7891.77");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=789.89,7876.01");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=61.71,7861.25");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=393.92,7852.46");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=469.77,7850.93");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=996.46,7870.57");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=108.46,7801.60");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=171.76,7801.31");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=236.51,7805.35");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=323.88,7792.02");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=386.24,7786.70");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=514.51,7806.96");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=621.82,7822.81");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=705.94,7794.39");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=865.09,7808.57");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=964.13,7803.83");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=9.51,7755.64");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=205.26,7745.16");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=450.05,7743.84");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=564.06,7743.27");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=639.57,7741.31");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=768.69,7765.00");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=38.43,7701.52");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=268.45,7740.86");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=332.36,7711.27");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=820.69,7714.35");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=895.02,7706.97");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=981.11,7728.18");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=131.66,7683.47");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=406.49,7688.21");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=499.51,7678.64");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=655.80,7676.67");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=747.22,7663.79");
    }

}

