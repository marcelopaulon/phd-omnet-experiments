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

network DADCA_ACK_n50_a8000m2_lpnFn1_0
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
        @display("i=misc/sensor;p=156.88,150.25");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=158.87,245.57");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=177.22,74.23");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=122.55,44.25");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=263.12,144.46");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=284.00,221.10");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=372.18,266.55");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=274.83,318.66");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=178.75,324.34");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=189.45,398.56");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=223.49,239.29");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=95.90,278.32");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=98.28,180.18");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=24.67,170.75");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=10.20,22.29");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=344.33,186.05");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=440.49,113.15");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=450.57,176.21");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=447.75,248.55");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=396.79,321.28");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=334.64,354.36");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=552.88,126.01");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=505.45,15.78");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=584.03,60.27");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=276.39,388.78");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=307.21,480.22");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=338.14,566.39");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=402.39,528.87");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=474.17,556.81");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=409.17,629.37");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=484.00,631.94");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=568.04,681.13");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=545.52,623.37");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=485.36,794.87");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=491.23,872.29");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=383.77,776.90");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=355.73,851.81");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=296.03,843.38");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=235.62,812.66");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=180.50,846.18");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=107.23,789.02");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=36.98,718.10");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=52.72,653.25");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=77.41,597.94");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=155.22,544.32");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=120.56,671.74");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=190.49,659.09");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=262.25,605.70");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=223.91,558.39");
    }

}

