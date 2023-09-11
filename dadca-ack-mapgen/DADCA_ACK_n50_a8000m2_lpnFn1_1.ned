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
        @display("i=misc/sensor,p=103.62,22.32");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=171.64,126.58");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=192.27,52.53");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=33.99,125.80");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=174.60,215.65");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=101.69,183.62");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=58.18,246.01");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=257.86,158.36");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=240.89,259.89");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=331.09,274.92");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=373.07,324.87");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=439.24,250.41");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=408.00,193.47");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=365.44,116.38");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=293.24,69.72");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=306.96,198.13");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=305.73,343.31");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=221.51,374.56");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=262.29,444.83");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=383.82,40.83");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=477.15,69.19");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=522.51,115.38");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=527.28,11.78");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=431.85,137.80");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=167.89,320.10");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=129.31,265.75");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=587.88,66.51");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=690.71,32.06");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=753.58,28.40");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=647.49,158.66");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=654.57,93.24");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=658.87,225.92");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=727.33,200.48");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=574.82,155.58");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=570.35,247.62");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=518.98,300.51");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=510.74,234.45");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=31.93,34.49");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=26.82,364.16");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=0.47,281.29");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=264.49,16.92");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=146.12,417.06");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=86.44,401.01");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=184.04,473.73");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=245.65,518.47");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=245.09,591.20");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=332.82,575.80");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=324.71,16.89");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=74.75,315.12");
    }

}

