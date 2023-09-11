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
        @display("i=misc/sensor;p=100.00,100.00");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=168.26,157.11");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=126.72,4.56");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=205.92,14.06");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=227.48,110.12");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=285.40,137.11");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=356.18,142.21");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=430.83,130.02");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=488.93,85.49");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=555.74,139.96");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=627.20,154.70");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=713.81,141.79");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=676.83,52.99");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=752.87,77.87");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=784.51,21.78");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=843.03,1.85");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=847.10,98.59");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=901.46,50.85");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=796.47,153.45");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=753.25,259.01");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=671.88,305.26");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=622.44,218.40");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=682.40,210.46");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=545.02,16.54");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=576.41,75.10");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=610.28,295.27");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=655.42,379.75");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=566.61,383.26");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=601.43,475.70");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=491.97,529.28");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=537.05,572.65");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=390.03,565.42");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=484.66,664.66");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=319.54,584.05");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=365.66,656.94");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=461.58,594.44");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=504.34,731.69");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=418.57,707.22");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=292.69,704.02");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=245.79,635.95");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=196.78,693.51");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=200.32,761.98");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=109.30,781.23");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=40.63,840.70");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=176.09,889.87");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=119.19,853.82");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=72.40,726.42");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=23.89,777.64");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=24.43,625.69");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=80.12,666.76");
    }

}

