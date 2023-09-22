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

network DADCA_ACK_n50_a8000m2_5
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
        @display("i=misc/sensor;p=142.76,7993.89");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=264.08,7994.04");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=361.21,7979.61");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=456.72,7997.98");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=660.17,7970.21");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=938.84,7990.44");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=23.14,7945.51");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=144.35,7930.29");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=218.23,7923.34");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=422.32,7932.44");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=500.28,7923.08");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=565.55,7934.77");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=754.35,7943.94");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=15.93,7876.79");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=75.30,7903.15");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=369.11,7891.69");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=680.88,7909.29");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=905.13,7903.05");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=986.43,7905.98");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=159.10,7840.49");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=227.67,7842.93");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=389.36,7828.28");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=468.38,7867.30");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=550.32,7849.29");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=633.26,7859.48");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=753.56,7844.53");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=845.17,7849.44");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=996.97,7832.27");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=35.48,7807.54");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=297.71,7820.22");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=468.71,7791.52");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=695.58,7816.12");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=946.58,7791.10");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=75.40,7744.14");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=173.06,7763.37");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=350.01,7742.82");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=526.08,7754.18");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=583.57,7778.81");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=643.12,7750.13");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=784.11,7771.64");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=285.97,7719.81");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=590.39,7718.15");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=753.12,7710.93");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=840.80,7732.81");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=915.61,7733.50");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=29.12,7672.51");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=174.39,7695.26");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=386.15,7692.00");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=487.28,7689.54");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=559.16,7661.50");
    }

}

