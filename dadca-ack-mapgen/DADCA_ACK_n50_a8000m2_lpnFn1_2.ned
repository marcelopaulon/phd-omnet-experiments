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

network DADCA_ACK_n50_a8000m2_lpnFn1_2
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
        @display("i=misc/sensor;p=125.24,13.70");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=179.97,87.73");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=246.96,106.89");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=196.42,23.44");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=259.60,39.25");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=337.75,2.36");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=420.03,8.95");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=441.39,129.41");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=380.78,95.69");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=401.35,251.60");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=332.44,261.34");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=250.69,209.71");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=170.21,168.09");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=164.87,264.89");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=26.33,48.34");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=66.11,152.13");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=40.92,231.92");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5.77,320.69");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=104.89,337.61");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=105.50,252.14");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=501.10,26.95");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=565.82,54.41");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=514.59,115.66");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=586.62,164.18");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=667.43,124.13");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=744.77,109.30");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=736.78,44.33");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=799.21,62.42");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=879.86,77.71");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=813.99,131.88");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=698.75,189.00");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=758.38,259.16");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=704.55,301.44");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=635.38,299.96");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=551.31,246.88");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=640.98,43.06");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=629.47,211.88");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=310.08,193.14");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=319.65,99.71");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=392.29,171.61");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=43.30,398.36");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=112.37,468.34");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=171.96,344.92");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=157.22,413.79");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=8.30,524.02");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=52.71,579.79");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=79.96,641.29");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=170.63,636.30");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=137.98,759.46");
    }

}

