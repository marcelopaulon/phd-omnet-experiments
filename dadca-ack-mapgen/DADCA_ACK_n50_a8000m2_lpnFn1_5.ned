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

network DADCA_ACK_n50_a8000m2_lpnFn1_5
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
        @display("i=misc/sensor,p=153.36,54.94");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=168.59,123.88");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=229.56,155.94");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=294.30,135.27");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=283.97,210.87");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=306.21,61.57");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=359.13,142.04");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=357.16,220.05");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=335.75,313.97");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=215.74,62.93");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=216.96,263.97");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=146.42,268.04");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=78.63,247.75");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=49.75,155.80");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=10.27,217.76");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=90.39,359.28");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=108.34,426.92");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=188.07,484.97");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=185.65,394.97");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=241.21,371.15");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=329.17,387.73");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=438.17,305.75");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=501.30,276.91");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=558.37,345.87");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=637.85,353.98");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=537.69,451.69");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=617.47,444.35");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=704.61,462.14");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=693.91,550.53");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=781.14,552.17");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=727.96,607.17");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=772.74,484.58");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=749.77,416.35");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=880.75,426.41");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=843.50,358.00");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=666.06,254.20");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=590.37,268.62");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=628.76,192.62");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=563.71,134.58");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=738.11,300.22");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=642.77,62.22");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=488.13,157.13");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=548.33,67.27");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=519.13,10.54");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=460.29,54.81");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=725.04,121.68");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=775.84,170.85");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=696.89,9.78");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=385.96,66.99");
    }

}

