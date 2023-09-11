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

network DADCA_ACK_n50_a8000m2_lpnFn1_7
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
        @display("i=misc/sensor,p=11.77,87.15");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=146.78,16.49");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=211.19,22.64");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=178.22,85.17");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=345.97,74.81");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=420.10,102.80");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=389.98,184.14");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=514.62,139.86");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=454.45,208.32");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=537.03,212.04");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=580.31,167.36");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=595.56,264.28");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=540.81,311.36");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=527.64,386.17");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=614.45,339.78");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=652.42,420.44");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=711.80,377.31");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=785.20,393.22");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=737.12,438.51");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=806.37,488.47");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=819.76,562.82");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=864.28,504.28");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=894.06,577.36");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=836.35,654.45");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=756.89,616.30");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=736.25,305.53");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=765.29,235.39");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=820.88,188.46");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=825.16,262.64");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=666.49,286.14");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=600.38,499.13");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=514.28,469.16");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=437.68,449.87");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=457.80,547.12");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=404.44,612.69");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=436.27,700.31");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=466.95,779.09");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=556.37,764.21");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=619.72,801.03");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=516.80,707.71");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=584.30,648.34");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=621.50,592.07");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=653.95,529.53");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=543.50,524.90");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=418.66,392.59");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=377.31,441.87");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=471.70,327.22");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=311.50,476.76");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=298.89,543.65");
    }

}

