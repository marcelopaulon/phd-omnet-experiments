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

network DADCA_ACK_n50_a8000m2_lpnFn1_6
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
        @display("i=misc/sensor,p=61.46,160.45");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=96.52,212.16");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=183.99,180.11");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=28.14,266.41");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=17.49,206.79");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=59.09,32.93");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=12.60,85.57");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=217.05,55.77");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=300.26,13.03");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=388.02,43.22");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=456.22,34.31");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=518.80,79.97");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=550.34,163.19");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=461.60,129.11");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=442.42,218.67");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=424.94,276.10");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=346.32,271.39");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=381.61,326.67");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=339.04,393.68");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=391.46,432.80");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=457.33,378.01");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=360.12,526.70");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=268.85,493.51");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=231.06,577.81");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=213.27,664.97");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=298.15,614.19");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=160.41,606.14");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=378.07,682.73");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=415.52,760.98");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=481.30,725.12");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=524.93,654.28");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=588.77,684.83");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=628.09,764.52");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=659.18,670.00");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=730.14,714.44");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=790.70,716.12");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=740.33,781.30");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=719.31,869.15");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=685.45,921.71");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=676.90,997.26");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=787.19,996.34");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=812.94,920.55");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=746.74,924.44");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=864.19,857.98");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=827.48,772.37");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=804.64,829.51");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=986.24,923.84");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=909.69,911.57");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=946.50,973.39");
    }

}

