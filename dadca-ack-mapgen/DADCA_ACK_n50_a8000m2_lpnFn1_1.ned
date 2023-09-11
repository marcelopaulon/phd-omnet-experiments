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
        @display("i=misc/sensor,p=60.33,30.05");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=121.62,210.94");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=43.00,224.21");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=13.55,160.63");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=37.63,342.69");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=97.32,291.36");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=124.53,377.65");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=59.17,423.48");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=101.91,497.33");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=217.80,341.05");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=231.02,424.46");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=303.40,386.35");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=320.75,325.64");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=392.49,357.89");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=187.46,246.06");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=300.71,461.25");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=255.87,512.69");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=325.87,544.61");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=369.95,494.86");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=407.59,447.49");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=169.17,512.54");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=218.53,562.45");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=213.33,665.55");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=273.75,677.73");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=344.10,724.42");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=300.87,795.59");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=390.26,685.82");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=458.02,693.93");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=458.08,633.63");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=554.82,614.05");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=613.31,629.12");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=645.34,691.48");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=563.07,516.80");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=512.79,432.96");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=528.59,343.76");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=461.41,311.02");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=573.19,277.00");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=641.02,256.66");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=569.72,196.22");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=548.53,108.79");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=493.65,169.43");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=418.32,206.96");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=394.41,150.09");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=512.35,2.71");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=432.90,13.69");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=358.00,58.34");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=289.71,59.42");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=313.65,164.37");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=257.33,243.32");
    }

}

